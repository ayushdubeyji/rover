import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', key_filename=os.path.expanduser('~/.ssh/id_ed25519') if os.path.exists(os.path.expanduser('~/.ssh/id_ed25519')) else None, password=os.environ.get('SSH_PASSWORD'), timeout=10)

sftp = c.open_sftp()
setup_path = '/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/setup.py'

with sftp.open(setup_path, 'r') as f:
    text = f.read().decode('utf-8')

old_str = "(os.path.join('share', package_name, 'web/static'), glob('web/static/*')),"
new_str = """(os.path.join('share', package_name, 'web/static'), [f for f in glob('web/static/*') if os.path.isfile(f)]),\n        (os.path.join('share', package_name, 'web/static/js'), glob('web/static/js/*')),"""

if old_str in text:
    text = text.replace(old_str, new_str)
    with sftp.open(setup_path, 'w') as f:
        f.write(text.encode('utf-8'))
    print("Successfully patched setup.py!")
else:
    print("old_str not found in setup.py")

sftp.close()
c.close()
