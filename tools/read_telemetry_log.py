import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=5)

cmd = "cat /tmp/telemetry_server.log; ss -tulpn | grep 8090"
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
err = stderr.read().decode()
if err:
    print("ERR:", err)
c.close()
