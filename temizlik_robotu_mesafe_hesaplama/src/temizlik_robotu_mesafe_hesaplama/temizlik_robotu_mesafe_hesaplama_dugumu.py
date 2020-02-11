#!/usr/bin/env python
# coding=utf-8

import rospy

from nav_msgs.msg import Odometry
from temizlik_robotu_msgs.msg import MesafeBilgisi
import math


#__________________________________________________________________________________________________
#
#  TemizlikRobotuMesafeHesaplama
#
#  AMAÇ:    Robotun toplam aldığı mesafeyi hesaplamak için
#           oluşturulmuş sınıftır.
#
#  SINIF DEĞİŞKENLERİ:
#
# 	### Constructor değişkenleridir. İlk ve varsayılan değerleri
#   ### aktarılmaktadır.
#
#   self.onceki_pozisyon_x
#   self.onceki_pozisyon_y
#   self.mevcut_pozisyon_x
#   self.mevcut_pozisyon_y
#   self.odom_okunma_kontrolu
#   self.toplam_mesafe
#
#
#
#  FONKSİYON PROTOTİPLERİ:
#
# 	// Constructor
# 	def __init__(self):
#
# 	def ana_fonksiyon(self):
#   def odom_callback_fonksiyonu(self, odom_mesaji):
#   def mesafe_hesaplama_fonksiyonu(self):
#
#
#
#   NOTLAR:
#
#   GELİŞTİRME GEÇMİŞİ:
#
#   Yazar:
#   Tarih: 10.02.2020
#   Versiyon: v_1.0
#
#
#__________________________________________________________________________________________________

class TemizlikRobotuMesafeHesaplama(object):

    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              __init__
    #   FONKSIYON AÇIKLAMASI:       Constructor fonksiyodur. Bütün değişkenlere
    #    							ilk ve varsayılan değerleri atanmaktadır.
    #
    #
    #   DEĞİŞKENLER:
    #       ADI                                     TIPI            AÇIKLAMASI
    #   onceki_pozisyon_x                           float           Mevcut zamandan bir önceki Odom topiği değerinin pozisyonun x değerini ifade eder.
    #   onceki_pozisyon_y                           float           Mevcut zamandan bir önceki Odom topiği değerinin pozisyonun y değerini ifade eder.
    #   mevcut_pozisyon_x                           float           Mevcut zamandaki Odom topiğini dinleyerek pozisyonun x değerini ifade eder.
    #   mevcut_pozisyon_y                           float           Mevcut zamandaki Odom topiğini dinleyerek pozisyonun y değerini ifade eder.
    #   odom_okunma_kontrolu                        bool            Odom topiğinin okunduğunu ifade eder. Odom topiği ilk okunduğunda bu değişken
    #                                                               True olur.
    #   toplam_mesafe                               float           Mevcut pozisyon ve onceki pozisyon değerleri arasındaki mesafe farkını ekleyerek
    #                                                               hareket boyunca toplam mesafe değerini ifade eder. 
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def __init__(self):
        self.onceki_pozisyon_x = 0
        self.onceki_pozisyon_y = 0        
        self.mevcut_pozisyon_x = 0
        self.mevcut_pozisyon_y = 0
        self.odom_okunma_kontrolu = False

        self.toplam_mesafe = 0

        self.ana_fonksiyon()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              ana_fonksiyon
    #   FONKSIYON AÇIKLAMASI:       Yayıncı dinlenerek toplam_mesafe değeri hesaplanır ve 
    #                               "/mesafe_hesabi" topiğinden değer yayınlanmaktadır.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "/odom" topiğine abone olabilmesi gerekmektedir.
    #
    #______________________________________________________________________________________________

    def ana_fonksiyon(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback_fonksiyonu)
        mesafe_hesabi_yayini = rospy.Publisher('/mesafe_hesabi', MesafeBilgisi, queue_size=10)
        
        # Odom topiği saniyede 10 defa yayınlandığı, hesaplanan mesafe değerini doğru bir şekilde hesaplayabilmek için
        # Saniyede 10 defa "while not rospy.is_shutdown()" döngüsü içindeki işlemleri gerçekleştirmektedir.
        rate = rospy.Rate(10)

        mesafe_hesap_mesaji = MesafeBilgisi()

        while not rospy.is_shutdown():
            # İlk odom topiği okunduktan sonra hesaplama işlemi gerçeklenmektedir.
            # Çünkü varsayılan değerler yerine kullanılacak gerçek değerler değişkenlere aktarılmıştır. 
            if self.odom_okunma_kontrolu:
                # Döngüye her girdiğinde mesafe_hesaplama_fonksiyonu tetiklenerek robotu aldığı mesafe
                # toplam_mesafe değişkenine eklenmektedir.
                self.toplam_mesafe += self.mesafe_hesaplama_fonksiyonu()                
                mesafe_hesap_mesaji.mesafe_bilgisi = self.toplam_mesafe
                rospy.loginfo(mesafe_hesap_mesaji)
                mesafe_hesabi_yayini.publish(mesafe_hesap_mesaji)

            rate.sleep()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              odom_callback_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       "Odom" yayıncısı dinlenerek okunan pozisyon x ve y değerleri sırasıyla mevcut_pozisyon_x ve
    #                               mevcut_pozisyon_y değerlerine aktarılmaktadır. Bu değişkenler değişmeden önceki değerleri sırasıyla
    #                               onceki_pozisyon_x ve onceki_pozisyon_y değişkenlerine aktarılmaktadır. Böylece hareket edilen mesafeyi
    #                               hesaplayabilmekteyiz.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   odom_mesaji                               Odometry                Topiğin değerinin okunması için gereken mesaj tipidir.  
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #   GEREKLILIK:
    #   Bu fonksiyon çağrılabilmesi için "/odom" topiğine abone olmak gerekmektedir. 
    #
    #______________________________________________________________________________________________

    def odom_callback_fonksiyonu(self, odom_mesaji):
        self.onceki_pozisyon_x = self.mevcut_pozisyon_x
        self.onceki_pozisyon_y = self.mevcut_pozisyon_y
        self.mevcut_pozisyon_x = odom_mesaji.pose.pose.position.x
        self.mevcut_pozisyon_y = odom_mesaji.pose.pose.position.y

        # İlk okunmada ilk değeri atamak için mevcut pozisyon verilir
        if not self.odom_okunma_kontrolu:
            self.onceki_pozisyon_x = odom_mesaji.pose.pose.position.x
            self.onceki_pozisyon_y = odom_mesaji.pose.pose.position.y

        # Odom okundu diye işaretler
        self.odom_okunma_kontrolu = True


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              mesafe_hesaplama_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Mevcuz pozisyon ve önceki pozisyon değerlerini kullanarak anlık alınan
    #                               mesafe değerini hesaplamak için kullanılan fonksiyondur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   hesaplanan_mesafe                           float                   Anlık hesaplanan mesafe değerini ifade eder.
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def mesafe_hesaplama_fonksiyonu(self):
        hesaplanan_mesafe = math.sqrt(math.pow(self.mevcut_pozisyon_x - self.onceki_pozisyon_x, 2) + math.pow(self.mevcut_pozisyon_y - self.onceki_pozisyon_y, 2))

        return hesaplanan_mesafe


if __name__ == '__main__':
    try:
        rospy.init_node('temizlik_robotu_mesafe_hesaplama_dugumu', anonymous=True)

        # TemizlikRobotuMesafeHesaplama() sınıfını çağırmaktadır.
        dugum = TemizlikRobotuMesafeHesaplama()

    except rospy.ROSInterruptException:
        pass
