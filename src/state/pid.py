import physics as p

class pid:
    def __init__(self,x,y,angle,ball):
        self.bot = p.robot(x=x,y=y,ball = ball,yaw=angle)
        prod = 0.055
        self.kpp = 0.1
        self.kpv = prod/self.kpp
        self.kpr = 0.02

    def gtg(self,xtg,ytg,bot,ball,thtg=0):
        if bot.dribble == 1:
            self.kpr = 0.01
        else:
            self.kpr = 0.02
        self.bot = bot
        vrx = (self.kpp * (xtg - self.bot.x))
        vry = (self.kpp * (ytg - self.bot.y))
        kx = (self.kpv * (vrx / 10 - self.bot.xd))
        ky = (self.kpv * (vry / 10 - self.bot.yd))
        while(thtg<0):
            if thtg<0:
                thtg = 6.28+thtg
            else:
                break
        while(thtg>6.28):
            if thtg>6.28:
                thtg = thtg - 6.28
            else:
                break
        if (abs(thtg-bot.theta)>0.1 and thtg <= 6.28 and thtg >= 0):
            if(abs(thtg-bot.theta)<(6.28-abs(thtg-bot.theta))):
                f = (thtg-bot.theta)/abs(thtg-bot.theta)
                e = abs(thtg-bot.theta)
                thetad = (e*self.kpr)*(f)
            else:
                f = (thtg-bot.theta)/abs(thtg-bot.theta)
                e = 6.28 - abs(thtg-bot.theta)
                thetad = -((e*self.kpr))*(f)        
        else:
            thetad = 0
        bot.movebot(kx=kx,ky=ky,ball = ball,thetad=thetad)

