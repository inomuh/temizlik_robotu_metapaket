# temizlik_robotu

TEMIZLIK ROBOTU Metapackage


İlk önce interactive marker twist server indirmek gerekmektedir. 

$ sudo apt-get install ros-kinetic-interactive-marker-twist-server

Workspace yok ise workspace oluşturunuz.

Örn. http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Daha sonra workspace gidiniz ve paketi indiriniz.

cd <WORKSPACE_NAME>

git clone "https://github.com/inomuh/temizlik_robotu.git"

gedit ~/.bashrc

export GAZEBO_MODEL_PATH=~/<WORKSPACE_NAME>/src/temizlik_robotu/:$GAZEBO_MODEL_PATH

cd <WORKSPACE_NAME>

catkin_make

catkin_make install

source devel/setup.bash

$ roslaunch temizlik_robotu_baslat temizlik_robotu_baslat.launch

Görev tamamlama yüzdesini görmek için

$ rosrun temizlik_robotu_bilgi_servisi temizlik_robotu_bilgi_servisi_baslat.launch
