#!/usr/bin/env python3
"""pexpect 驱动 ssh 到香橙派执行命令（自动处理 ssh 密码 + sudo 密码）

用法:
    python3 /tmp/pi.py 'uname -a'
    python3 /tmp/pi.py --file /tmp/script.sh
    PI_HOST=orangepi@192.168.1.100 python3 /tmp/pi.py '...'
"""
import base64
import os
import shlex
import sys

import pexpect

HOST = os.environ.get('PI_HOST', 'orangepi@192.168.31.209')
PW = os.environ.get('PI_PW', 'orangepi')


def run(remote, timeout=900):
    cmd = 'ssh -tt -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ' \
          '-o ConnectTimeout=12 -o LogLevel=ERROR %s %s' % (HOST, shlex.quote(remote))
    ch = pexpect.spawn(cmd, encoding='utf-8', timeout=timeout, codec_errors='replace')
    parts = []
    sent = 0
    while True:
        try:
            i = ch.expect([r'(?i)\[sudo\] password', r'(?i)password:',
                           pexpect.EOF, pexpect.TIMEOUT], timeout=timeout)
        except pexpect.EOF:
            parts.append(ch.before or '')
            break
        except pexpect.TIMEOUT:
            parts.append(ch.before or '')
            parts.append('\n__TIMEOUT__')
            break
        parts.append(ch.before or '')
        if i in (0, 1):
            if sent >= 4:
                parts.append('\n__TOO_MANY_PASSWORD_PROMPTS__')
                break
            ch.sendline(PW)
            sent += 1
            continue
        break
    try:
        ch.close(force=True)
    except Exception:
        pass
    return ''.join(parts)


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == '--file':
        with open(sys.argv[2], 'rb') as f:
            b64 = base64.b64encode(f.read()).decode()
        remote = 'echo %s | base64 -d | bash' % shlex.quote(b64)
    else:
        remote = ' '.join(sys.argv[1:])
    sys.stdout.write(run(remote))
