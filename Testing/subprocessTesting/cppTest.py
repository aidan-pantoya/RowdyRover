import subprocess

cmd = "pyTest.cxx"
subprocess.call(["g++", cmd, "-o", "pyTest.out"])
subprocess.call("./pyTest.out")
subprocess.call(["rm", "pyTest.out"])
