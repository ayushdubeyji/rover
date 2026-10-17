import sys, paramiko
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=15)
is_sudo = ('sudo' in sys.argv[1:])
args = [a for a in sys.argv[1:] if a != 'sudo']
cmd = " ".join(args)
if is_sudo:
    cmd = "echo REMOVED | sudo -S -p '' bash -lc " + repr(cmd)
_, o, e = c.exec_command(cmd, timeout=120, get_pty=False)
out_bytes = o.read()
err_bytes = e.read()
sys.stdout.buffer.write(out_bytes[-6000:])
sys.stderr.buffer.write(err_bytes[-2000:])
c.close()
