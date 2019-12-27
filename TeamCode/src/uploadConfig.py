import os, subprocess

for config_file in os.listdir('config'):
    print(f'Uploading {config_file}')
    subprocess.run(['adb', 'push', 'config/'+config_file, '/sdcard/FIRST/config/'])