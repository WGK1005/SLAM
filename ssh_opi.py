#!/usr/bin/env python3
import sys
import pexpect

HOST = "10.207.70.100"
USER = "orangepi"
PASS = "orangepi"
KEY_PUB = "/home/wgk/scripts/keys/opi_id_ed25519.pub"
KEY_PRIV = "/home/wgk/scripts/keys/opi_id_ed25519"

def install_key():
    with open(KEY_PUB, "r") as f:
        pub_content = f.read().strip()
    
    # We will ssh and append the pub key to ~/.ssh/authorized_keys
    cmd = f"ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null {USER}@{HOST} 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo \"{pub_content}\" >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo KEY_INSTALLED_OK'"
    child = pexpect.spawn(cmd, timeout=15)
    
    idx = child.expect([pexpect.TIMEOUT, pexpect.EOF, "password:", "Password:"])
    if idx in (2, 3):
        child.sendline(PASS)
        idx2 = child.expect([pexpect.TIMEOUT, pexpect.EOF, "KEY_INSTALLED_OK"])
        if "KEY_INSTALLED_OK" in child.before.decode(errors='ignore') or idx2 == 2:
            print("SSH key installation successful!")
            return True
        else:
            print("Failed after sending password:", child.before.decode(errors='ignore'))
            return False
    else:
        print("Unexpected prompt or timeout:", child.before.decode(errors='ignore'))
        return False

if __name__ == "__main__":
    if install_key():
        print("Success! Can now connect with private key.")
    else:
        sys.exit(1)
