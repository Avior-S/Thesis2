import subprocess, shlex, psutil
import time

def start_record(cmd):
    cmd = shlex.split(cmd)
    rosbag_proc = subprocess.Popen(cmd)
    return cmd, rosbag_proc

def end_record(cmd, rosbag_proc):
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(cmd[2:]).issubset(proc.cmdline()):
            proc.send_signal(subprocess.signal.SIGINT)

    rosbag_proc.send_signal(subprocess.signal.SIGINT)

if __name__ == "__main__" :
    cmd, rosbag_proc = start_record("rosbag record -a -b 1 -o bag_records/panda_test")
    time.sleep(20)
    end_record(cmd,rosbag_proc)