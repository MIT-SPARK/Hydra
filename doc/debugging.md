## Debugging Hydra

Hydra may crash (i.e. segfault) on systems that we haven't tested on. This is typically due to a invalid configuration of third-party libraries. When this happens, there are several ways to get more information about what might be going on (to either debug yourself or to create a more informative bug report).

### Running hydra with GDB to get a backtrace

The most helpful piece of information is to get a backtrace of where a segfault is occurring or other issue is occurring.
Hydra should be set up to output a backtrace when it crashes, but this may not always appear (it depends on how far the Hydra node gets in the startup process).
You may have to install GDB (`sudo apt install gdb`) if you haven't already. You can then run:

```
roslaunch hydra_ros uhumans2.launch debug:=true
```

which runs `gdb` against the main Hydra node by setting the launch prefix.
You can look at other useful launch prefixes [here](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB).

It might take a minute or so for `gdb` to load the multi-thread debugging tools). Once Hydra fully starts up, start the rosbag (or other data source) that you are using, and wait until the crash is reproduced. Entering `bt` into the prompt should get you a similar backtrace to the example one below:

```
Thread 1 "hydra_dsg_build" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
51      ../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) bt
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
#1  0x00007ffff5a547f1 in __GI_abort () at abort.c:79
#2  0x00007ffff6f234e7 in ?? () from /usr/lib/x86_64-linux-gnu/libglog.so.0
#3  0x00007ffff6f1a0cd in google::LogMessage::Fail() () from /usr/lib/x86_64-linux-gnu/libglog.so.0
#4  0x00007ffff6f1bf33 in google::LogMessage::SendToLog() () from /usr/lib/x86_64-linux-gnu/libglog.so.0
#5  0x00007ffff6f19c28 in google::LogMessage::Flush() () from /usr/lib/x86_64-linux-gnu/libglog.so.0
#6  0x00007ffff6f1c999 in google::LogMessageFatal::~LogMessageFatal() () from /usr/lib/x86_64-linux-gnu/libglog.so.0
#7  0x00007ffff521cb83 in kimera::SemanticLabel2Color::SemanticLabel2Color (this=0x555555811340, filename=...) at /home/ubuntu/catkin_ws/src/kimera_semantics/kimera_semantics/src/color.cpp:51
#8  0x00007ffff7b1dbbb in hydra::incremental::DsgFrontend::DsgFrontend (this=0x7fffffffcb40, nh=..., dsg=...) at /home/ubuntu/catkin_ws/src/hydra/hydra_dsg_builder/src/incremental_dsg_frontend.cpp:58
#9  0x0000555555560bfe in run (nh=..., frontend_dsg=std::shared_ptr<hydra::incremental::SharedDsgInfo> (use count 3, weak count 0) = {...},
    backend_dsg=std::shared_ptr<hydra::incremental::SharedDsgInfo> (use count 2, weak count 0) = {...}, output_path="/home/ubuntu/catkin_ws/src/hydra/hydra_dsg_builder/output/uhumans2/office")
    at /home/ubuntu/catkin_ws/src/hydra/hydra_dsg_builder/src/incremental_dsg_builder_node.cpp:56
#10 0x000055555556061d in main (argc=<optimized out>, argv=<optimized out>) at /home/ubuntu/catkin_ws/src/hydra/hydra_dsg_builder/src/incremental_dsg_builder_node.cpp:123
(gdb)
```

### Checking the workspace

It is always good to double-check your catkin workspace configuration:
```
roscd
catkin config
```
and the repository information from your workspace:
```
roscd && cd ..
vcs export src > workspace_info.yaml
```
to make sure they align with the information in the installation instructions.

### Checking the build

It is occasionally useful to double-check what catkin and cmake are actually linking and included when building a package.
Cleaning the package and then building (e.g. `catkin clean hydra_dsg_builder && catkin build hydra_dsg_builder --no-deps -v`) and looking at the invocations to `gcc` will provide the most information for this.
Building with a single thread (i.e. `catkin build hydra_dsg_builder --no-deps -v -j 1`) will also make sure the output is in order (at the expense of build time).

### Building with debug symbols

You can enable debug symbols by changing the flag `-DCMAKE_BUILD_TYPE=Release` to `-DCMAKE_BUILD_TYPE=RelWithDebInfo` and rebuilding.
You can remove and add flags to the catkin config by using `-r` and `-a` respectively (e.g. `catkin config -a --cmake-flags -DCMAKE_BUILD_TYPE=Release`). This will add line numbers to any backtrace you create using gdb.
