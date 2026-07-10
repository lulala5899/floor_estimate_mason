#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
import time
import math
import os
import sys
import site
import asyncio
import concurrent.futures

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
from std_msgs.msg import Int32, Float32

from barometer_interfaces.msg import Barometer, ZMotion

import yaml

# Add virtual environment site-packages to sys.path if running in a virtual environment
if venv := os.environ.get('VIRTUAL_ENV'):
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    if os.path.isdir(site_packages := os.path.join(venv, 'lib', pyver, 'site-packages')) and site_packages not in sys.path:
        site.addsitedir(site_packages)
# Keep the import order for virtual environment compatibility
import aiohttp
from aiohttp import web
import serial_asyncio
import serial
import serial.tools.list_ports


def find_serial_port() -> str:
    """
    Find the first available serial port that matches the ESP32 device with a barometer.
    """
    sysname = os.uname().sysname.lower()
    serial_ports = None
    if 'darwin' in sysname:  # macOS
        serial_ports = [p.device for p in serial.tools.list_ports.comports()
                        if "usbserial" in p.device]
    elif 'linux' in sysname:
        serial_ports = [p.device for p in serial.tools.list_ports.comports()
                        if ("ttyUSB" in p.device or "ttyACM" in p.device)]
    else:
        serial_ports = [p.device for p in serial.tools.list_ports.comports()]

    accessible_ports = []
    for port in serial_ports:
        try:
            with serial.Serial(port, 115200, timeout=2) as ser:
                accessible_ports.append(port)
                for _ in range(5):
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Reading from {port}: {line}")
                    if line.startswith('BARO'):
                        return port
        except (serial.SerialException, OSError):
            continue

    # Fallback: if no BARO-prefixed frames are observed, return the first
    # accessible candidate port instead of failing fast.
    if accessible_ports:
        return accessible_ports[0]

    return None


class PressureNode(Node):
    def __init__(self):
        super().__init__('esp32_serial_baro')

        # Cache logger methods for better performance and readability
        self._info = self.get_logger().info
        self._warn = self.get_logger().warning
        self._error = self.get_logger().error
        self._debug = self.get_logger().debug

        # Initialize async serial connection (will be set up in async context)
        self.serial_reader = None
        self.serial_writer = None
        self.connection_attempts = 0
        self.max_connection_attempts = 5
        self.reconnect_delay = 2.0  # seconds

        default_params = self._load_default_parameters_from_file()
        self.cali_offsets = default_params.get('cali_offsets', {})
        self._warned_unknown_base_macs = set()

        self._prev_h = None       # previous altitude (meters)
        self._prev_t = None       # previous time (seconds)
        self._prev_v = 0.0        # previous vertical speed (m/s)

        # Baseline calibration state
        self.calibration_start_time = None
        self.calibration_pressures = []
        self.baseline_complete = False
        self.current_floor = 0

        self.pub_pressure = self.create_publisher(
            FluidPressure, '/pressure', 10)
        self.pub_barometer = self.create_publisher(
            Barometer, '/barometer', 10)
        self.pub_zmotion = self.create_publisher(
            ZMotion, '/z_motion', 10)
        self.pub_baseline_pressure = self.create_publisher(
            Float32, '/baseline_pressure', 10)
        self.pub_floor_estimate = self.create_publisher(
            Int32, '/floor_estimate', 10)

        self.sub_barometer = self.create_subscription(
            Barometer, '/barometer', self._sub_barometer_callback, 10)

        self.declare_parameter('serial_port',
                               default_params.get('serial_port', ''),
                               ParameterDescriptor(description='Serial port for ESP32 connection'))
        self.serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        if not self.serial_port:
            self.serial_port = find_serial_port()
            if not self.serial_port:
                self._error(
                    'No serial port found for ESP32(Barometer) connection. Please check connection.')
                raise RuntimeError('No serial port found')
            self._info(f'Serial port set to: {self.serial_port}')

        self.declare_parameter('output_mode',
                               default_params.get(
                                   'output_mode', 'self-relative'),
                               ParameterDescriptor(description='self-relative or base-relative output mode'))
        self.output_mode = self.get_parameter(
            'output_mode').get_parameter_value().string_value

        self.declare_parameter(
            'frequency',
            default_params.get('frequency', 6.0),
            ParameterDescriptor(
                description='Sensor data frequency in Hz')
        )
        self.frequency = self.get_parameter(
            'frequency').get_parameter_value().double_value

        if self.output_mode == "base-relative":
            self.declare_parameter(
                'base_ip',
                default_params.get('base_ip', ''),
                ParameterDescriptor(
                    description='IP address of the base station to fetch base pressure data from')
            )
            self.base_ip = self.get_parameter(
                'base_ip').get_parameter_value().string_value
            self.pub_base_pressure = self.create_publisher(
                FluidPressure, '/base/pressure', 10)
            self.pub_base_barometer = self.create_publisher(
                Barometer, '/base/barometer', 10)

            self.declare_parameter(
                'base_pressure_port',
                default_params.get('base_pressure_port', 18080),
                ParameterDescriptor(
                    description='HTTP port for receiving base pressure POST /data')
            )
            self.base_pressure_port = self.get_parameter(
                'base_pressure_port').get_parameter_value().integer_value

        self.declare_parameter(
            'default_local_pressure',
            default_params.get('default_local_pressure', 1010.0),
            ParameterDescriptor(
                description='Sea-level reference pressure in hPa')
        )
        self.default_local_pressure = self.get_parameter(
            'default_local_pressure').get_parameter_value().double_value

        self.declare_parameter(
            'calibration_duration',
            10.0,
            ParameterDescriptor(
                description='Seconds to collect pressure data on startup for UG baseline calibration')
        )
        self.calibration_duration = self.get_parameter(
            'calibration_duration').get_parameter_value().double_value

        self.declare_parameter(
            'floor_height',
            3.0,
            ParameterDescriptor(
                description='Height of each floor in meters, for floor estimation')
        )
        self.floor_height = self.get_parameter(
            'floor_height').get_parameter_value().double_value

        self.declare_parameter(
            'longitude',
            default_params.get('longitude', 121.60),
            ParameterDescriptor(
                description='Longitude for local pressure fetching')
        )
        self.declare_parameter(
            'latitude',
            default_params.get('latitude', 31.68),
            ParameterDescriptor(
                description='Latitude for local pressure fetching')
        )
        self.longitude = self.get_parameter(
            'longitude').get_parameter_value().double_value
        self.latitude = self.get_parameter(
            'latitude').get_parameter_value().double_value

        self.mac_address: str | None = None
        self.mac_timer = self.create_timer(1.0, self._send_mac_address)
        self.pressure_offset = 0.0
        self.temperature_offset = 0.0

        # Add server-related attributes
        self.base_pressure_server = None
        self.server_runner = None

        self._info('PressureNode has been started.')

    def _load_default_parameters_from_file(self):
        """
        Loads default parameters from a YAML file within the package.
        Centralized management of developer-defined defaults.
        """
        try:
            share_directory = get_package_share_directory('serial_to_ros2')
            config_file_path = os.path.join(
                share_directory, 'config', 'esp32_serial_baro.yaml')

            if os.path.exists(config_file_path):
                with open(config_file_path, 'r') as f:
                    full_config = yaml.safe_load(f)
                    node_params = full_config.get(
                        self.get_name()).get('ros__parameters', {})
                    self._info(
                        f"Successfully loaded default parameters from {config_file_path}")
                    return node_params
            else:
                self._warn(
                    f"Default parameter file not found at {config_file_path}. Using hardcoded fallbacks.")
        except Exception as e:
            self._error(
                f"Failed to load default parameters from file: {e}. Using hardcoded fallbacks.")
        return {}

    async def setup_serial_connection(self):
        """Setup async serial connection with retry logic"""
        for attempt in range(self.max_connection_attempts):
            try:
                # Retry finding serial port if connection failed
                if not self.serial_port or attempt > 0:
                    self._info("Searching for ESP32 device...")
                    found_port = find_serial_port()
                    if found_port:
                        self.serial_port = found_port
                        self._info(f'Found ESP32 at: {self.serial_port}')
                    else:
                        self._warn('No ESP32 device found')
                        if attempt < self.max_connection_attempts - 1:
                            self._info(
                                f'Retrying in {self.reconnect_delay} seconds...')
                            await asyncio.sleep(self.reconnect_delay)
                            continue
                        else:
                            raise RuntimeError(
                                'No ESP32 device found after all attempts')

                self.serial_reader, self.serial_writer = await serial_asyncio.open_serial_connection(
                    url=self.serial_port,
                    baudrate=115200
                )
                self._info(
                    f'Serial connection established at {self.serial_port}')
                self.connection_attempts = 0
                return True

            except Exception as e:
                self._error(
                    f'Failed to establish serial connection (attempt {attempt + 1}): {e}')
                if attempt < self.max_connection_attempts - 1:
                    self._info(
                        f'Retrying in {self.reconnect_delay} seconds...')
                    await asyncio.sleep(self.reconnect_delay)
                else:
                    raise

        return False

    async def close_serial_connection(self):
        """Close serial connection safely"""
        try:
            if self.serial_writer:
                self.serial_writer.close()
                await self.serial_writer.wait_closed()
        except Exception as e:
            self._warn(f"Error closing serial connection: {e}")
        finally:
            self.serial_reader = None
            self.serial_writer = None

    def _send_mac_address(self):
        """Send MAC address request with serial to identify which barometer is connected"""
        if self.serial_writer is None:
            self._debug(
                "Serial writer not available, skipping MAC address request")
            return

        try:
            data = "WHICH_MAC>\n".encode("utf-8")
            self.serial_writer.write(data)
            self._debug("Sent MAC address request")
        except Exception as e:
            self._error(f"Failed to send MAC address request: {e}")

    def _sub_barometer_callback(self, msg: Barometer):
        # Current altitude and timestamp
        h_now = msg.altitude
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Skip if altitude is invalid
        if self._prev_h is None:
            # First message received, initialize altitude, velocity, and acceleration to 0
            self._prev_h = h_now
            self._prev_t = t_now
            self._prev_v = 0.0
            self._info('First altitude: %.3f m' % h_now)
            return

        dt = t_now - self._prev_t
        if dt <= 0.0:
            # Skip if timestamp is invalid
            return

        # Compute current vertical velocity
        v_now = (h_now - self._prev_h) / dt

        # Compute vertical acceleration
        a_now = (v_now - self._prev_v) / dt

        # Log or publish velocity and acceleration
        self._debug(
            f'Alt: {h_now:.3f} m | Vel: {v_now:.3f} m/s | Acc: {a_now:.3f} m/s²')
        self.pub_zmotion.publish(
            ZMotion(
                header=msg.header,
                vspeed=v_now,
                vacc=a_now
            )
        )
        # Update previous values
        self._prev_h = h_now
        self._prev_t = t_now
        self._prev_v = v_now

    def _send_time_sync(self):
        """Send time sync timestamp to esp32 to sync with host machine"""
        if self.serial_writer is None:
            self._debug("Serial writer not available, skipping time sync")
            return

        try:
            timestamp = int(time.time() * 1000)
            data = f"TS>{timestamp}\n".encode("utf-8")
            self.serial_writer.write(data)
            self._debug(f"Sent time sync: {timestamp}")
        except Exception as e:
            self._error(f"Failed to send time sync: {e}")

    def _serial_raw_to_pressure(self, line: str):
        try:
            data = re.split(r',\s*', line[6:])  # Skip 'BAROD>' prefix
            if len(data) < 3:
                self._warn(f'Invalid data format: {line}')
                return None
            
            try:
                pressure_from_sensor = float(data[1])
                temp_from_sensor = float(data[2])
                if math.isnan(pressure_from_sensor) or \
                    math.isinf(pressure_from_sensor) or \
                    pressure_from_sensor < 980.0 or \
                    pressure_from_sensor > 1100.0 or \
                    math.isnan(temp_from_sensor) or \
                    math.isinf(temp_from_sensor) or \
                    temp_from_sensor < -20.0 or \
                    temp_from_sensor > 50.0:
                    self._warn(f"Received invalid sensor data from serial: {line}")
                    return None
            except ValueError:
                self._warn(f"Could not parse float from sensor data in line: {line}")
                return None
        
            timestamp = float(data[0])
            # Convert timestamp from milliseconds to ROS2 time
            timestamp_sec = int(timestamp // 1000)
            timestamp_nanosec = int((timestamp % 1000) * 1000000)

            header = Header()
            header.stamp.sec = timestamp_sec
            header.stamp.nanosec = timestamp_nanosec
            header.frame_id = "barometer_link"
            
            # Pressure in Pascals
            pressure_hpa = pressure_from_sensor + self.pressure_offset
            pressure_pa = pressure_hpa * 100.0
            temperature = temp_from_sensor + self.temperature_offset

            # --- Baseline calibration phase ---
            if not self.baseline_complete:
                if self.calibration_start_time is None:
                    self.calibration_start_time = time.time()
                    self._info(f"Starting baseline calibration: collecting pressure for {self.calibration_duration}s")

                elapsed = time.time() - self.calibration_start_time
                self.calibration_pressures.append(pressure_hpa)
                self._debug(f"Calibration sample {len(self.calibration_pressures)}: {pressure_hpa:.2f} hPa")

                if elapsed >= self.calibration_duration:
                    self.default_local_pressure = sum(self.calibration_pressures) / len(self.calibration_pressures)
                    self.baseline_complete = True
                    self._info(
                        f"Baseline calibration done: {self.default_local_pressure:.2f} hPa "
                        f"(UG layer, {len(self.calibration_pressures)} samples over {elapsed:.1f}s)")
                    self.pub_baseline_pressure.publish(Float32(data=float(self.default_local_pressure)))

                # During calibration, still publish raw pressure (without altitude)
                self.pub_pressure.publish(
                    FluidPressure(
                        header=header,
                        fluid_pressure=pressure_pa,
                        variance=0.0
                    )
                )
                return

            # --- Normal operation (after calibration) ---
            altitude = self._altitude_from_pressure(pressure_hpa, temperature)

            if math.isnan(altitude) or math.isinf(altitude):
                self._warn(f"Calculated invalid altitude (NaN/inf). Skipping publish. "
                        f"Input P={pressure_hpa:.2f}, T={temperature:.2f}, "
                        f"BaseP={self.default_local_pressure:.2f}")
                return None

            self.pub_pressure.publish(
                FluidPressure(
                    header=header,
                    fluid_pressure=pressure_pa,
                    variance=0.0
                )
            )
            self.pub_barometer.publish(
                Barometer(
                    header=header,
                    pressure=pressure_pa,
                    temperature=temperature,
                    altitude=altitude
                )
            )

            # --- Floor estimate ---
            self.current_floor = int(round(altitude / self.floor_height))
            self.pub_floor_estimate.publish(Int32(data=self.current_floor))
        except Exception as e:
            self._warn(f'Parse error: {e}')
            return None

    def _get_mac_address(self, line: str) -> str:
        """
        Extracts the MAC address from the given line.
        """
        try:
            return line[6:].strip().replace(':', '_')  # Skip 'MAC>' prefix
        except Exception as e:
            self._error(f"Failed to extract MAC address: {e}")
            return None

    def _altitude_from_pressure(self,
                                P: float,
                                T_celsius: float,
                                L: float = 0.0065,
                                R: float = 287.05,
                                g: float = 9.80665) -> float:
        """
        Calculate altitude based on the barometric formula in the troposphere
        according to the International Standard Atmosphere (ISA) model.

        Parameters:
        - P        : Actual atmospheric pressure (hPa)
        - T_celsius: Actual temperature (°C)
        - L        : Temperature lapse rate, default is 0.0065 K/m
        - R        : Specific gas constant for dry air, default is 287.05 J/(kg·K)
        - g        : Gravitational acceleration, default is 9.80665 m/s²

        Returns:
        - h        : Altitude (meters)

        Example:
        >>> altitude_from_pressure(950.0, 15.0)
        561.554...
        """
        T = T_celsius + 273.15  # Convert °C to K
        exponent = (R * L) / g
        h = (T / L) * (1.0 - (P / self.default_local_pressure) ** exponent)
        return h

    def _process_serial_line(self, line: str) -> None:
        """Process a line received from serial - extracted from async callback"""
        if line.startswith('BAROD>'):
            self._serial_raw_to_pressure(line)
            return
        elif line.startswith('BAROT>'):
            # Handle time sync message
            self._send_time_sync()
            return
        elif line.startswith('BAROM>'):
            # Handle MAC address request
            self.mac_address = self._get_mac_address(line)
            self._info(f'Received MAC address: {self.mac_address}')
            # Stop the MAC address timer if it exists
            if self.mac_timer is not None:
                self.destroy_timer(self.mac_timer)
                self.mac_timer = None

            # use the MAC address to identify connected barometer's calibration offsets
            if self.mac_address in self.cali_offsets:
                offsets = self.cali_offsets[self.mac_address]
                self.pressure_offset = offsets.get('pressure_offset', 0.0)
                self.temperature_offset = offsets.get(
                    'temperature_offset', 0.0)
                self._info(f'Got MAC {self.mac_address}, offsets P={self.pressure_offset} hPa, '
                           f'T={self.temperature_offset} °C')
            else:
                self._warn(f'Unknown MAC {self.mac_address}, use zero offsets')
            return
        else:
            self._info(f'ESP32: {line}')
            return

    async def serial_reader_task(self):
        """Async task to continuously read from serial with auto-reconnection"""
        try:
            while True:
                try:
                    # Check if connection exists
                    if self.serial_reader is None:
                        self._info("Serial connection not available, attempting to connect...")
                        success = await self.setup_serial_connection()
                        if not success:
                            self._error("Failed to establish connection, retrying...")
                            await asyncio.sleep(self.reconnect_delay)
                            continue

                    # Try to read from serial
                    line_bytes = await asyncio.wait_for(
                        self.serial_reader.readline(),
                        timeout=5.0  # 5 second timeout
                    )

                    if not line_bytes:
                        # Empty read might indicate disconnection
                        self._warn("Empty read from serial, checking connection...")
                        await self.close_serial_connection()
                        continue

                    line = line_bytes.decode('utf-8').strip()
                    if line:
                        self._process_serial_line(line)

                except asyncio.TimeoutError:
                    # Timeout reading from serial - device might be disconnected
                    self._warn("Serial read timeout, checking connection...")
                    await self.close_serial_connection()
                    continue

                except (serial.SerialException, OSError, ConnectionResetError) as e:
                    # Serial connection error - device disconnected
                    self._warn(f"Serial connection lost: {e}")
                    await self.close_serial_connection()
                    self._info(f"Waiting {self.reconnect_delay} seconds before reconnection attempt...")
                    await asyncio.sleep(self.reconnect_delay)
                    continue

                except Exception as e:
                    self._error(f"Unexpected error in serial reader: {e}")
                    await self.close_serial_connection()
                    await asyncio.sleep(self.reconnect_delay)
                    continue
                    
        except asyncio.CancelledError:
            self._info("Serial reader task cancelled")
            raise  # Re-raise to properly handle the cancellation
    
    async def _handle_pressure_post(self, request):
        """Handle POST requests with pressure data"""
        try:
            # Parse JSON data from request
            data = await request.json()
            
            # Get Device-Mac from request headers
            device_mac_header = request.headers.get('Device-Mac')
            device_mac = device_mac_header.replace(":", "_").upper() if device_mac_header else ""
            if device_mac:
                self._debug(f"Received request from device MAC: {device_mac}")
            else:
                self._warn("No Device-Mac header found in request")
                
            # Extract data from JSON
            timestamp_ms = data.get('timestamp_ms', 0)
            
            try:
                pressure_hpa = data.get('pressure_hpa', 0.0)
                temperature_c = data.get('temperature_c', 0.0)
                if math.isnan(pressure_hpa) or \
                    math.isinf(pressure_hpa) or \
                    pressure_hpa < 980.0 or \
                    pressure_hpa > 1100.0 or \
                    math.isnan(temperature_c) or \
                    math.isinf(temperature_c) or \
                    temperature_c < -20.0 or \
                    temperature_c > 50.0:
                    self._warn(f"Received invalid sensor data from base post: {data}")
                    return web.Response(
                        text="Base pressure data received successfully",
                        status=500)
            except ValueError:
                self._warn(f"Could not parse float from sensor data in line: {data}")
                return web.Response(
                        text="Could not parse float from sensor data",
                        status=500)
            
            # Convert timestamp from milliseconds to ROS2 time
            timestamp_sec = int(timestamp_ms // 1000)
            timestamp_nanosec = int((timestamp_ms % 1000) * 1000000)
            
            # Create ROS2 message header
            header = Header()
            header.stamp.sec = timestamp_sec
            header.stamp.nanosec = timestamp_nanosec
            header.frame_id = "base_barometer_link"
            
            if device_mac and device_mac in self.cali_offsets:
                offsets = self.cali_offsets[device_mac]
                pressure_offset = offsets.get('pressure_offset', 0.0)
                temperature_offset = offsets.get('temperature_offset', 0.0)
            else:
                pressure_offset = 0.0
                temperature_offset = 0.0
                if device_mac and device_mac not in self._warned_unknown_base_macs:
                    self._warn(f"Unknown MAC {device_mac} in base POST, using zero offsets")
                    self._warned_unknown_base_macs.add(device_mac)
            
            # Convert pressure from hPa to Pa and create FluidPressure message
            pressure_hpa = float(pressure_hpa) + pressure_offset
            pressure_pa = pressure_hpa * 100.0
            temperature = float(temperature_c) + temperature_offset
            altitude = self._altitude_from_pressure(pressure_hpa, temperature)
            
            # Publish base pressure data
            self.pub_base_pressure.publish(
                FluidPressure(
                    header=header,
                    fluid_pressure=pressure_pa,
                    variance=0.0
                )
            )
            self.pub_base_barometer.publish(
                Barometer(
                    header=header,
                    pressure=pressure_pa,
                    temperature=temperature,
                    altitude=altitude
                )
            )
            
            # Return success response
            return web.Response(
                text="Base pressure data received successfully",
                status=200)
            
        except Exception as e:
            self._error(f"Error processing pressure data: {e}")
            return web.json_response(
                {"status": "error", "message": str(e)},
                status=500
            )

    async def _setup_base_pressure_server(self, port: int):
        """Setup the base pressure HTTP server"""
        try:
            # Create web application
            app = web.Application()
            
            # Add routes
            app.router.add_post('/data', self._handle_pressure_post)
            
            # Add CORS middleware
            # async def add_cors_headers(request, handler):
            #     response = await handler(request)
            #     response.headers['Access-Control-Allow-Origin'] = '*'
            #     response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
            #     response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
            #     return response
            
            # app.middlewares.append(add_cors_headers)
            
            # Setup runner
            self.server_runner = web.AppRunner(app)
            await self.server_runner.setup()
            
            # Start server
            site = web.TCPSite(self.server_runner, '0.0.0.0', port)
            await site.start()
            
            self._info(f"Base pressure server started on port {port}")
            self._info("Endpoints: POST /data (receive data)")

            return True
            
        except Exception as e:
            self._error(f"Failed to setup base pressure server: {e}")
            return False

    async def _cleanup_base_pressure_server(self):
        """Cleanup the base pressure server"""
        try:
            if self.server_runner:
                await self.server_runner.cleanup()
                self.server_runner = None
                self._info("Base pressure server cleaned up")
        except Exception as e:
            self._warn(f"Error cleaning up base pressure server: {e}")

    async def fetch_base_pressure_start(self):
        """Start the base pressure server"""
        try:
            # Setup server
            success = await self._setup_base_pressure_server(self.base_pressure_port)
            if not success:
                self._error("Failed to start base pressure server")
                return
            
            self._info("Base pressure server started successfully")
            # Server is now running in the background, task can complete
            
        except Exception as e:
            self._error(f"Error starting base pressure server: {e}")
            await self._cleanup_base_pressure_server()
            raise

    async def _register_ip_for_base_server(self) -> None:
        """Register the IP address of the base server"""
        if not self.base_ip:
            self._info("base_ip is empty; skipping direct /clientip registration")
            return
        timeout = aiohttp.ClientTimeout(total=10)
        try:
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.post(f"http://{self.base_ip}/clientip") as resp:
                    self._info(f"HTTP POST status: {resp.status}")
                    if resp.status == 200:
                        data = await resp.json()
                        if data.get("status") == "success":
                            self._info(f"Base server IP registered successfully: {data.get('your_client_ip')}")
                    if resp.status != 200:
                        self._warn(f"Failed to register base server IP")
                        return
        except Exception as e:
            self._error(f"Error registering base server IP: {e}")
            return
        
    async def fetch_local_pressure(self) -> float:
        """
        Asynchronously fetches local(ShanghaiTech University) mean sea level pressure (MSL pressure) in hPa
        from the Open-Meteo API for the given latitude and longitude.

        Parameters:
            latitude (float): Latitude,
            longitude (float): Longitude,

        Returns:
            float: Pressure in hPa, or float('nan') if failed
        """
        url = (
            f"https://api.open-meteo.com/v1/forecast"
            f"?latitude={self.latitude}&longitude={self.longitude}&current=pressure_msl"
        )
        headers = {
            "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) "
            "AppleWebKit/537.36 (KHTML, like Gecko) "
            "Chrome/135.0.0.0 Safari/537.36",
            "Accept": "application/json",
        }

        timeout = aiohttp.ClientTimeout(total=10)
        try:
            self._info(f"Fetching MSL pressure from {url}")
            # — Potential optimization: reuse a ClientSession across multiple calls
            async with aiohttp.ClientSession(headers=headers, timeout=timeout) as session:
                async with session.get(url) as resp:
                    self._info(f"MSL HTTP GET status: {resp.status}")
                    if resp.status != 200:
                        text = await resp.text()
                        self._warn(f"Unexpected status {resp.status}: {text}")
                        return float("nan")

                    data = await resp.json()
                    pressure = data.get("current", {}).get("pressure_msl")
                    if isinstance(pressure, (int, float)) and not math.isnan(pressure):
                        return float(pressure)

                    self._error( "Invalid or missing 'pressure_msl' in response JSON")
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._error(f"Error fetching local pressure: {e}")

        return float("nan")


async def async_main(args=None):
    """Async main function that handles both ROS2 and serial communication"""
    rclpy.init(args=args)
    node = PressureNode()
    executor = None
    executor_task = None
    serial_task = None

    try:
        # Setup serial connection
        await node.setup_serial_connection()
        # Create executor
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Create executor task
        loop = asyncio.get_event_loop()
        executor_task = loop.run_in_executor(
            concurrent.futures.ThreadPoolExecutor(max_workers=1),
            executor.spin
        )
        # Create serial reader task
        serial_task = asyncio.create_task(node.serial_reader_task())

        # In-situ baseline calibration is done in _serial_raw_to_pressure on startup,
        # so skip the Open-Meteo API fetch. default_local_pressure from YAML is used
        # as a temporary fallback until calibration (10s) completes.
        node._info("Baseline pressure will be calibrated from sensor data on startup (10s)")
            
        if node.output_mode == "base-relative":
            # Start base pressure server task
            base_pressure_task = asyncio.create_task(node.fetch_base_pressure_start())
            
            # Register IP for base server if output mode is base-relative
            register_ip_task = asyncio.create_task(node._register_ip_for_base_server())
            
            # Wait for all tasks
            await asyncio.gather(executor_task, register_ip_task, serial_task, base_pressure_task)
        else:
            # Wait for both tasks (original behavior)
            await asyncio.gather(executor_task, serial_task)

    except (KeyboardInterrupt, asyncio.CancelledError):
        node._info("Received shutdown signal, cleaning up...")
        
        # Cancel tasks gracefully
        if serial_task and not serial_task.done():
            serial_task.cancel()
            try:
                await asyncio.wait_for(serial_task, timeout=2.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass
        
        # Cancel base pressure task if running
        if 'base_pressure_task' in locals() and base_pressure_task and not base_pressure_task.done():
            base_pressure_task.cancel()
            try:
                await asyncio.wait_for(base_pressure_task, timeout=2.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass
                
        if executor_task and not executor_task.done():
            executor_task.cancel()
            try:
                await asyncio.wait_for(executor_task, timeout=2.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass
    
    except Exception as e:
        node._error(f"Error in async_main: {e}")
    finally:
        # Clean up serial connection
        await node.close_serial_connection()
        
        # Clean up base pressure server
        await node._cleanup_base_pressure_server()
        
        # Clean up ROS2 resources
        try:
            if executor:
                executor.shutdown()
            node.destroy_node()
        except Exception as e:
            node._warn(f"Error during node cleanup: {e}")
            
        # Only shutdown if not already shut down
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # Suppress RClPy shutdown errors as they're often harmless
            pass


def main(args=None):
    """Synchronous wrapper for the async main function"""
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Application error: {e}")
        return 1
    return 0


if __name__ == '__main__':
    main()
