import subprocess
revision=(subprocess.check_output(["git", "describe", "--abbrev=6", "--always", "--tags"]).strip().decode("utf-8"))
print("-DGITVERSION=\"{}\"".format(revision))
#echo "-DGITVERSION=\"$(git describe --abbrev=4 --dirty --always --tags)\""