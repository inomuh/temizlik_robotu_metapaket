import rospy
from temizlik_robotu_supurme_dugumu import TemizlikRobotuSupurme


robotum = TemizlikRobotuSupurme()

gorev_listesi = list([[2, 0], [5, 90], [1, 180], [5,90], [1, 0], [5,90], [1, 180], [5,90], [1, 0], [5,90], [1, 180], [5,0]])
robotum.gorev_atama_func(gorev_listesi)
robotum.main_func()