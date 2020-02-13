#!/usr/bin/env python
# coding=utf-8

import rospy

from sensor_msgs.msg import Range
from temizlik_robotu_msgs.srv import *


#__________________________________________________________________________________________________
#
#  TemizlikRobotuEngeldenSakinma
#
#  AMAÇ:    Robotun engelden sakınması ve hareket emrini vermesi için
#           oluşturulmuş sınıftır.
#
#  SINIF DEĞİŞKENLERİ:
#
# 	### Constructor değişkenleridir. İlk ve varsayılan değerleri
#   ### aktarılmaktadır.
#
#   self.robot_hareket_emir
#   self.onceki_robot_hareket_emir_durumu
#   self.robot_hareket_emir_degisim_kontrolu
#   self.engel_menzil_limit_degeri
#
#
#
#  FONKSİYON PROTOTİPLERİ:
#
# 	// Constructor
# 	def __init__(self):
#
# 	def ana_fonksiyon(self):
#   def sonar_callback_fonksiyonu(self, menzil_mesaji):
#   def engel_tespiti_istemcisi(self, istek):
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

class TemizlikRobotuEngeldenSakinma(object):

    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              __init__
    #   FONKSIYON AÇIKLAMASI:       Constructor fonksiyodur. Bütün değişkenlere
    #    							ilk ve varsayılan değerleri atanmaktadır.
    #
    #
    #   DEĞİŞKENLER:
    #       ADI                                     TIPI            AÇIKLAMASI
    #   robot_hareket_emir                          bool            Mevcut zamandaki Sonar topiğini dinleyerek engelin olup olmadığını ifade eder. Bu değer False ise engelin var olduğunu belirtir.
    #   onceki_robot_hareket_emir_durumu            bool            Bir önceki robot_hareket_emir durumunu ifade eder.
    #   robot_hareket_emir_degisim_kontrolu         bool            robot_hareket_emir ve onceki_robot_hareket_emir_durumu karşılaştırarak durumun değişip değişmediğini ifade eder.
    #                                                               Eğer değişim var ise "engel_bilgi" servisine istek yollar.
    #   engel_menzil_limit_degeri                   float           Parametreler yaml dosyasından okunmuş Engel Menzil Limit Degeri değerini ifade eder.
    #                                                               Sonar topiğinden okunan mesafe bilgisinin engel olarak belirleyebilmek için kullanılan limit değerdir.
    #                                                               Eğer okunan değer bu değerin altında ise robot_hareket_emir değeri True olur.
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
        self.robot_hareket_emir = True
        self.onceki_robot_hareket_emir_durumu = True        
        self.robot_hareket_emir_degisim_kontrolu = False

        self.engel_menzil_limit_degeri = rospy.get_param("~Parametreler/engel_menzil_limit_degeri")

        self.ana_fonksiyon()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              ana_fonksiyon
    #   FONKSIYON AÇIKLAMASI:       Yayıncı dinlenerek okunan robot_hareket_emir değeri robot_hareket_emir_degisim_kontrolu
    #                               değeri True olduğunda "engel_bilgi" servisine istek olarak gönderilmektedir.
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
    #   Bu fonksiyon kullanılabilmesi için "/sonar1" topiğine abone olabilmesi gerekmektedir.
    #
    #______________________________________________________________________________________________

    def ana_fonksiyon(self):
        rospy.Subscriber('/sonar1', Range, self.sonar_callback_fonksiyonu)
        
        # Saniyede 2 defa "while not rospy.is_shutdown()" döngüsü içindeki işlemleri gerçekleştirmektedir.
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.robot_hareket_emir_degisim_kontrolu:
                # Eğer önceki durum ile mevcut durum arasında fark varsa servisi çağırarak robot_hareket_emir değerini göndermektedir.
                sonuc = self.engel_tespiti_istemcisi(self.robot_hareket_emir)
                print("Engel Tespiti Servisini Cagir")
                self.robot_hareket_emir_degisim_kontrolu = False

            rate.sleep()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              sonar_callback_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       "Sonar1" yayıncısı dinlenerek okunan değer engel_menzil_limit_degeri ile karşılaştırılır.
    #                               Eğer bu değerden az ise robot_hareket_emir değeri False olur. robot_hareket_emir ve
    #                               onceki_robot_hareket_emir_durumu birbirine eşit değil ise robot_hareket_emir_degisim_kontrolu
    #                               True olur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   menzil_mesaji                               Range                   Topiğin değerinin okunması için gereken mesaj tipidir.  
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #   GEREKLILIK:
    #   Bu fonksiyon çağrılabilmesi için "/sonar1" topiğine abone olmak gerekmektedir. 
    #
    #______________________________________________________________________________________________

    def sonar_callback_fonksiyonu(self, menzil_mesaji):
        okunan_sonar_menzili = float(menzil_mesaji.range)

        if okunan_sonar_menzili < self.engel_menzil_limit_degeri:
            # Engelin olduğunu belli etmektedir ve robotun hareketinin sonlanması gerektiğini belirtir.
            self.robot_hareket_emir = False

        else:
            self.robot_hareket_emir = True

        # robot_hareket_emir ve onceki_robot_hareket_emir_durumu değerleri birbirine eşit değil ise
        # robot_hareket_emir_degisim_kontrolu True olur ve servise istek yollar.
        if self.robot_hareket_emir != self.onceki_robot_hareket_emir_durumu:
            self.robot_hareket_emir_degisim_kontrolu = True
            self.onceki_robot_hareket_emir_durumu = self.robot_hareket_emir


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              engel_tespiti_istemcisi
    #   FONKSIYON AÇIKLAMASI:       Robotun robot_hareket_emir değerini servis ile iletişim kurması için kullanılan
    #                               istemci fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   istek                                       EngelBilgi              Servise robot_hareket_emir değerini gönderir.
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   yanit                                       EngelBilgi              Servisten gönderilen yanit değerini alır.
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "engel_bilgi" servisinin açık olması gerekmektedir.
    #
    #______________________________________________________________________________________________

    def engel_tespiti_istemcisi(self, istek):
        rospy.wait_for_service('engel_bilgi')

        try:
            engel_bilgi = rospy.ServiceProxy('engel_bilgi', EngelBilgi)
            yanit = engel_bilgi.call(EngelBilgiRequest(istek))

            return yanit.yanit

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == '__main__':
    try:
        rospy.init_node('temizlik_robotu_engelden_sakinma_dugumu', anonymous=True)

        # TemizlikRobotuEngeldenSakinma() sınıfını çağırmaktadır.
        dugum = TemizlikRobotuEngeldenSakinma()

    except rospy.ROSInterruptException:
        pass
