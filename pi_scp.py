#!/usr/bin/env python3
"""pexpect 驱动 scp 到/从香橙派（免装 sshpass）

用法（src/dst 原样传给 scp，哪边带 orangepi@...: 哪边就是远端）:
    python3 /tmp/pi_scp.py <本地文件> orangepi@192.168.31.209:<远端目录>
    python3 /tmp/pi_scp.py orangepi@192.168.31.209:<远端文件> <本地目录>
    python3 /tmp/pi_scp.py -r orangepi@192.168.31.209:~/maps <本地目录>
"""
import os
import sys

import pexpect

PW = os.environ.get('PI_PW', 'orangepi')

args = sys.argv[1:]
if not args:
    print(__doc__)
    sys.exit(1)

cmd = ['scp', '-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=/dev/null',
       '-o', 'LogLevel=ERROR'] + args
print('运行: %s' % ' '.join(cmd), flush=True)

ch = pexpect.spawn(' '.join(cmd), encoding='utf-8', timeout=1800, codec_errors='replace')
for _ in range(4):
    i = ch.expect([r'(?i)password:', pexpect.EOF, pexpect.TIMEOUT], timeout=60)
    if i == 0:
        ch.sendline(PW)
        continue
    break
try:
    ch.expect(pexpect.EOF, timeout=1800)
except pexpect.TIMEOUT:
    print('__TIMEOUT__')
out = (ch.before or '').strip()
ch.close(force=True)
if out:
    print(out)
