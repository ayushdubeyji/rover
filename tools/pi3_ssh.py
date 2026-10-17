import sys
import paramiko

host = '192.168.0.103'
user = 'ayush'
pw = 'REMOVED'

cmd = sys.argv[1] if len(sys.argv) > 1 else 'uname -a'
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username=user, password=pw, timeout=10, allow_agent=False, look_for_keys=False)
stdin, stdout, stderr = c.exec_command(cmd, timeout=30)
out = stdout.read().decode('utf-8', 'ignore')
err = stderr.read().decode('utf-8', 'ignore')
if out:
    print(out, end='')
if err:
    print(err, file=sys.stderr, end='')
c.close()
