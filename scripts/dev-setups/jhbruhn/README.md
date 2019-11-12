# a very sophisticated VM

To use this dev-environment, you need vagrant und virtualbox. For linux, you can usually get both via your package manager, for windows and macos there is an installer available here: 
https://www.vagrantup.com/downloads.html

After installing, you just have to run this:

```
git clone https://github.com/arpg-sophisticated/dev-environment.git
cd dev-environment/
vagrant up
vagrant ssh
```

et voila, you are ssh-ed into your VM with all dependencies installed automagically.
To run the ROS-projects, follow the instructions of the code-repo (which was automatically cloned into your current directory).
