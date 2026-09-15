import pexpect, sys
cmd = sys.argv[1]
c = pexpect.spawn("ssh", ["-o", "StrictHostKeyChecking=accept-new", "ysc@192.168.1.120", cmd], timeout=20, encoding="utf-8")
i = c.expect(["assword:", "yes/no", pexpect.EOF, pexpect.TIMEOUT])
if i == 1:
    c.sendline("yes")
    i = c.expect(["assword:", pexpect.EOF, pexpect.TIMEOUT])
if i == 0:
    c.sendline("'")
    c.expect(pexpect.EOF, timeout=20)
print(c.before)
