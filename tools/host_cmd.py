import os
import sys
import paramiko

def run(cmd):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    host = '192.168.0.111'
    user = 'ayush'
    pw = os.environ.get('SSH_PASSWORD')
    key_path = os.path.expanduser('~/.ssh/id_ed25519')
    try:
        if os.path.exists(key_path):
            c.connect(host, username=user, key_filename=key_path, timeout=10)
        else:
            c.connect(host, username=user, password=pw, timeout=10)
    except Exception:
        if pw:
            c.connect(host, username=user, password=pw, timeout=10)
        else:
            raise
    stdin, stdout, stderr = c.exec_command(cmd, timeout=30)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    c.close()
    return out, err

if __name__ == '__main__':
    if len(sys.argv) > 1:
        cmd = " ".join(sys.argv[1:])
        out, err = run(cmd)
        if out: print(out, end='')
        if err: print(err, end='', file=sys.stderr)
