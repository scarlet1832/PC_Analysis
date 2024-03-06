import subprocess

# 启动print1.py并捕获输出
process = subprocess.Popen(['python', 'print1.py'], stdout=subprocess.PIPE)

while True:
    output = process.stdout.readline()
    if output == b'' and process.poll() is not None:
        break
    if output:
        print("Got new output:", output.decode())