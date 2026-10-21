import os
import paramiko

def connect_host(host='192.168.0.111', user='ayush', timeout=10):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    key_path = os.path.expanduser('~/.ssh/id_ed25519')
    pw = os.environ.get('SSH_PASSWORD')
    try:
        if os.path.exists(key_path):
            c.connect(host, username=user, key_filename=key_path, timeout=timeout)
        else:
            c.connect(host, username=user, password=pw, timeout=timeout)
    except Exception:
        if pw:
            c.connect(host, username=user, password=pw, timeout=timeout)
        else:
            raise
    return c
