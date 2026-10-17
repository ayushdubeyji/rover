import sys
import paramiko

def run(cmd):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=10)
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
