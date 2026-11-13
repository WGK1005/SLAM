#!/usr/bin/env python3
import sys
import pexpect

HOST = "192.168.31.111"
USER = "qrssys"
PASS = "123456"
KEY_PUB = "/home/wgk/scripts/keys/pc_id_ed25519.pub"
KEY_PRIV = "/home/wgk/scripts/keys/pc_id_ed25519"

def install_key():
    with open(KEY_PUB, "r") as f:
        pub_content = f.read().strip()
    
    cmd = (f"ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "
           f"-o ConnectTimeout=8 {USER}@{HOST} "
           f"'mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo \"{pub_content}\" >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo KEY_INSTALLED_OK'")
    
    print(f"Connecting to {USER}@{HOST}...")
    child = pexpect.spawn(cmd, encoding='utf-8', timeout=15)
    
    idx = child.expect([pexpect.TIMEOUT, pexpect.EOF, r'(?i)password:'])
    if idx == 2:
        child.sendline(PASS)
        idx2 = child.expect([pexpect.TIMEOUT, pexpect.EOF, "KEY_INSTALLED_OK"])
        output = child.before or ''
        if "KEY_INSTALLED_OK" in output or idx2 == 2:
            print("SSH key installation successful!")
            return True
        else:
            print("Failed after sending password:", output)
            return False
    elif idx == 1:
        print("Connected directly or closed:", child.before)
        if "KEY_INSTALLED_OK" in (child.before or ''):
            return True
        return False
    else:
        print("Timeout connecting to host:", child.before)
        return False

def test_key_connection():
    cmd = (f"ssh -i {KEY_PRIV} -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "
           f"-o ConnectTimeout=5 {USER}@{HOST} 'uname -a && whoami'")
    child = pexpect.spawn(cmd, encoding='utf-8', timeout=10)
    idx = child.expect([pexpect.TIMEOUT, pexpect.EOF])
    output = child.before or ''
    print("Test output:\n" + output.strip())
    return "qrssys" in output

if __name__ == "__main__":
    if install_key():
        print("Testing passwordless login...")
        if test_key_connection():
            print("ALL SET! Passwordless connection verified.")
            sys.exit(0)
    sys.exit(1)

