import os, sys, paramiko

host = '192.168.0.111'
user = 'ayush'
pw = os.environ.get('SSH_PASSWORD')
key_path = os.path.expanduser('~/.ssh/id_ed25519')

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    if os.path.exists(key_path):
        c.connect(host, username=user, key_filename=key_path, timeout=15)
    else:
        c.connect(host, username=user, password=pw, timeout=15)
except Exception:
    if pw:
        c.connect(host, username=user, password=pw, timeout=15)
    else:
        raise

is_sudo = ('sudo' in sys.argv[1:])
args = [a for a in sys.argv[1:] if a != 'sudo']
cmd = " ".join(args)
if is_sudo:
    if pw:
        cmd = f"echo {pw} | sudo -S -p '' bash -lc " + repr(cmd)
    else:
        cmd = "sudo bash -lc " + repr(cmd)
_, o, e = c.exec_command(cmd, timeout=300, get_pty=False)
out_bytes = o.read()
err_bytes = e.read()
sys.stdout.buffer.write(out_bytes[-6000:])
sys.stderr.buffer.write(err_bytes[-2000:])
c.close()
