#!/usr/bin/env python
# coding=utf-8

import rospy
from temizlik_robotu_msgs.srv import *


#__________________________________________________________________________________________________
#
#  TemizlikRobotuBilgiServisi
#
#  AMAÇ:    Robotun görevi tamamlamam bilgisini öğrenmek için
#           oluşturulmuş sınıftır.
#
#  SINIF DEĞİŞKENLERİ:
#
#
#
#  FONKSİYON PROTOTİPLERİ:
#
# 	// Constructor
# 	def __init__(self):
#
# 	def ana_fonksiyon(self):
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

class TemizlikRobotuBilgiServisi(object):

    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              __init__
    #   FONKSIYON AÇIKLAMASI:       Constructor fonksiyodur.
    #
    #
    #   DEĞİŞKENLER:
    #       ADI                                     TIPI            AÇIKLAMASI
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
        self.ana_fonksiyon()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              ana_fonksiyon
    #   FONKSIYON AÇIKLAMASI:       Öğrenilmek istenilen bilgi "bilgi_servisi" servisine istek olarak gönderilmektedir.
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
    #   Bu fonksiyon kullanılabilmesi için "bilgi_servisi" servisine istekte bulunması gerekmektedir.
    #
    #______________________________________________________________________________________________

    def ana_fonksiyon(self):
        # Öğrenilmek istenen bilgi bilgi_servisi_istemcisi fonksiyonuna parametre olarak gönderilmektedir.
        servisten_gelen_yanit = self.bilgi_servisi_istemcisi("Gorev Yuzdesi")
        print("\n\n\n")
        print("Tamamlanan Gorev Yuzdesi")
        print("\t % " + str(servisten_gelen_yanit))
        print("\n\n\n")


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              bilgi_servisi_istemcisi
    #   FONKSIYON AÇIKLAMASI:       Robotun görevi tamamlamam bilgisini öğrenmek için servis ile iletişim kurması
    #                               için kullanılan istemci fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   istek                                       BilgiServisi            Servise "Gorev Yuzdesi" değerini gönderir.
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   yanit                                       BilgiServisi            Servisten gönderilen görevin tamamlanan yüzde değerini
    #                                                                       yanit olarak alır.
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "engel_bilgi" servisinin açık olması gerekmektedir.
    #
    #______________________________________________________________________________________________

    def bilgi_servisi_istemcisi(self, istek):
        rospy.wait_for_service('bilgi_servisi')

        try:
            # Bilgi servisini oluşturur
            bilgi_servisi = rospy.ServiceProxy('bilgi_servisi', BilgiServisi)
            # Servise isteği parametre olarak yollar ve servis dönüşü yanit olarak gelir.
            yanit = bilgi_servisi.call(BilgiServisiRequest(istek))

            return yanit.yanit

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == '__main__':
    try:
        rospy.init_node('temizlik_robotu_bilgi_servisi_dugumu', anonymous=True)
        
        # TemizlikRobotuBilgiServisi() sınıfını çağırmaktadır.
        dugum = TemizlikRobotuBilgiServisi()

    except rospy.ROSInterruptException:
        pass
