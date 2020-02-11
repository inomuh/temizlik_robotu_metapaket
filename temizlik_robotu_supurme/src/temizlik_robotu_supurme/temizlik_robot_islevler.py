import rospy
from temizlik_robotu_supurme_dugumu import TemizlikRobotuSupurme



class RobotIslevler(TemizlikRobotuSupurme):
    
    def __init__(self):
        super.__init__(self)


    def donme_func(self, hedef_bas_aci_derece):
        """
        Derece cinsinden olan hedef baş açı değerini radyan cinsine çevrilir.
        
        Mevcut baş açı ile hedef baş açı arasındaki fark alınır ve açısal referans hız değeri 
        çarpılarak uygulanacak açısal hız değeri üretilir.

        Mevcut baş açı ile hedef baş açı arasındaki fark belirtilen baş açı tolerans değerinden küçük ise
        donus yapma fonksiyonu sonlanır ve diğer göreve geçer.

        """

        hedef_bas_aci_radyan = hedef_bas_aci_derece*math.pi/180         # Derece alma pi/2 tarzı al direk radyan al
        acisal_hiz = self.acisal_referans_hizi * (hedef_bas_aci_radyan - self.bas_aci)

        if abs(hedef_bas_aci_radyan - self.bas_aci) < self.bas_aci_tolerans_degeri:
            acisal_hiz = 0.0
            self.donus_yap = False
            #print("Mevcut Gorev Bitti Digerine Gec")
            #print(self.gorev_listesi[self.gorev_numarasi])
            #print("\n\n\n")
            self.gorev_numarasi += 1

        return acisal_hiz