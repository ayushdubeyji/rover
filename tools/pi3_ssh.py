import os
import sys
import paramiko

host = '192.168.0.103'
user = 'ayush'
pw = os.environ.get('PI_PASSWORD') or os.environ.get('SSH_PASSWORD')
key_path = os.path.expanduser('~/.ssh/id_ed25519')

cmd = sys.argv[1] if len(sys.argv) > 1 else 'uname -a'
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    if os.path.exists(key_path):
        c.connect(host, username=user, key_filename=key_path, timeout=10)
    elif pw:
        c.connect(host, username=user, password=pw, timeout=10, allow_agent=False, look_for_keys=False)
    else:
        c.connect(host, username=user, timeout=10)
except Exception:
    if pw:
        c.connect(host, username=user, password=pw, timeout=10, allow_agent=False, look_for_keys=False)
    else:
        raise
stdin, stdout, stderr = c.exec_command(cmd, timeout=30)
out = stdout.read().decode('utf-8', 'ignore')
err = stderr.read().decode('utf-8', 'ignore')
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.stderr.reconfigure(encoding='utf-8', errors='replace')
if out:
    print(out, end='')
if err:
    print(err, file=sys.stderr, end='')
c.close()
