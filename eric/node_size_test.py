#!/usr/bin/python
import subprocess

node_counts = [ "1", "2", "4", "8" ]
image_sizes = [ "128", "256", "512", "1024" ] # smaller sizes fail for some reason
ray_depths = [ "1", "2", "4", "8", "16", "32" ]

exe = "/home/03755/ericlee/gvt/gravit/build/bin/gvtMpi"
model = "/home/03755/ericlee/hman/data/simple"
name = "simple"
eye = "0 0 100"
look = "0 0 0"
light_pos = "0 50 10"
light_color = "450 450 450"
depth = "1"

if 1:
  subprocess.call("mkdir output", shell=True)
  for nodes in node_counts:
    print("using " + nodes + " nodes")
    for size in image_sizes:
      print("  image size -> " + size)
      subprocess.call("nodes=" + nodes + " && " +
                      "width=" + size + " && " +
                      "height=" + size + " && " +
                      "depth=" + depth + " && " +
                      "source simple_single_frame.sh &> /dev/null && " +
                      "mv prof_simple_async_domain_size_1_rank_0.ppm output/nodes_" + nodes + "_size_" + size + ".ppm", shell=True)
                      #"mv " + output_name + " output/nodes_" + nodes + "_size_" + size + ".ppm", shell=True) # don't know why this fails (some weird python behavior?)

if 1:
  print("")
  diff_bin = "~/gvt/gravit/build/bin/gvtImageDiff"

  for nodes in node_counts:
    if nodes == "1":
      continue
    print("============ " + nodes + " nodes ==============")
    for size in image_sizes:
      #subprocess.call(diff_bin + " -diff output/nodes_1_size_" + size + ".ppm,output/nodes_" + nodes + "_size_" + size + ".ppm", shell=True)
      subprocess.call(diff_bin + " -diff output/nodes_1_size_" + size + ".ppm,golden/golden_" + size + ".ppm", shell=True)
      print("")

# turn on secondary rays
if 1:
  print("TURNING ON SECONDARY RAYS (depth=8)")
  depth = "8"

  subprocess.call("mkdir output_secondary", shell=True)
  for nodes in node_counts:
    print("using " + nodes + " nodes")
    for size in image_sizes:
      print("  image size -> " + size)
      subprocess.call("nodes=" + nodes + " && " +
                      "width=" + size + " && " +
                      "height=" + size + " && " +
                      "depth=" + depth + " && " +
                      "source simple_single_frame.sh &> /dev/null && " +
                      "mv prof_simple_async_domain_size_1_rank_0.ppm output_secondary/nodes_" + nodes + "_size_" + size + "_secondary.ppm", shell=True)
                      #"mv " + output_name + " output/nodes_" + nodes + "_size_" + size + ".ppm", shell=True) # don't know why this fails (some weird python behavior?)

if 1:
  print("")
  diff_bin = "~/gvt/gravit/build/bin/gvtImageDiff"

  for nodes in node_counts:
    if nodes == "1":
      continue
    print("============ " + nodes + " nodes ==============")
    for size in image_sizes:
      subprocess.call(diff_bin + " -diff output_secondary/nodes_1_size_" + size + "_secondary.ppm,output_secondary/nodes_" + nodes + "_size_" + size + "_secondary.ppm", shell=True)
      print("")
 
