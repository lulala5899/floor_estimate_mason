import os
import shutil

Import("env")


feature_file = f"{env['PIOENV']}.cpp"

project_dir = env.subst("$PROJECT_DIR")
src_dir = os.path.join(project_dir, "src")
src_files = os.path.join(project_dir, "src_files")
dst_file = os.path.join(src_dir, "main.cpp")
src_file = os.path.join(src_files, feature_file)

print(f"Copying {src_file} to {dst_file}")
shutil.copy(src_file, dst_file)
