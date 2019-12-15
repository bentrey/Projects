from carRaceFunctions import *

x=0.0
v=0.0
t=0.0
dt=0.0001

#Fixed Parameters
carMass=1724
d=402.336
mus=0.72
areaOfSpoiler=0.15
cod=0.33
driveShaftMass=100
wheelMass=100
crossSectionalArea=3
h=1.5

#Variable Parameters
wheelBase=4.0
carLength=5.0
distanceBumperToAxel=1.0
engineLocation=0.6
engineType='LS'
spoilerAttackAngle=5.0
wheelWidth=215
wheelDiameter=15
aspectRatio=65

'''
#Engine########################################################

  LS3 (Corvette)
    Id=LS3
    Dyno
      BestFitCubic (rpm,hp)
        [(1750,100),(3100,200),(4300,300),(6000,390),(6500,370)]
    Weight
      195 kg

  Hemi Apache (Challenger)
    Id=HA
    Dyno
      BestFitCubic (rpm,hp)
        [(1000,200), (2000,350), (4000,350),(4400,400),(4950,450),
         (5350,500),(6250,530), (7100,500)]
    Weight
      318 kg

  5.0L GT (Mustang)
    Id=GT
    Dyno
      BestFitCubic (rpm,hp)
        [(1000,50),(2000,100),(3200,200),(4200,300),(5100,350),
         (6500,395)]
    Weight
      201 kg

#Transmission####################################################

  Borg-Warnder T56
    GearRatios={ 1:2.97, 2:1.94, 3:1.35, 4:1.00, 5:0.84, 6,0.62,
                 -1:3.38}

#Differential####################################################

  Differential Ratio
    3.83:1

#Spoiler#########################################################

  Coefficient of Lift
    Wind Tunnel
      BestFitCubic (degrees,Cl)
        [(0,0), (5,-0.4),(10,-0.8),(13,-1.0),(15,-0.9),(20,-0.85)]

  Coefficient of Drag
    Wind Tunnel
      BestFitCubic (degrees,Cd)
        [(5,0.035),(10,0.07),(15,0.16),(17,0.23),(20,0.33)]

#Specifications##################################################

  1. The front tire can not leave the ground.
  2. The car in the simulation will be driven perfectly with no
       tire slipping.
  3. Your tire and wheel size must be commerically available.
  4. Wheelbases are from 3.0m-4.0m.
  5. Car lengths are from 4.5m-5.5m.
  6. Distance from bumper to axle maybe 0.5m-1.5m.
  7. The Engine may be 0.5m-2.0m from the front bumper or
     0.5m-2.0m from the rear bumper.
  8. The engine position is measured from the front of the car.
  9. You must choose one of the given engines.
  10. The horse power will be determined by a cubic fit to the
     given data.
  11. The angle of attack for the spoiler may take values
     0-25 degrees.
  12. You may only change the variable parameters.
  13. The spoiler must be at the end of the car 1m above the axel.
  14. The height of the center of mass of the car body will be at
     the same as the engine and 0.2 cm above the axels.
'''

if abs(wheelBase-3.5)>0.5:
    print('Bad Wheelbase')
elif abs(carLength-5.0)>0.5:
    print('Bad Car Length')
elif abs(1.0-distanceBumperToAxel)>0.5:
    print('Bad Distance From Bumper to Axel')
elif not abs(1.25-engineLocation)>0.75 and not abs(carLength-1.25-engineLocation)>0.75:
    print('Bad Engine Location')
elif not engineType in ['LS','HA','GT']:
    print('Bad Engine Type')
elif abs(12.5-spoilerAttackAngle)>12.5:
    print('Bad Spoiler Attack Angle')
else:
    k=0.5*carMass+0.25*driveShaftMass+wheelMass
    if engineType=='LS':
        hpCurve=LS3curve
        em=195
    elif engineType=='GT':
        hpCurve=GTcurve
        em=201
    else:
        hpCurve=HAcurve
        em=318
    Fe=em*9.81
    Fb=carMass*9.81
    F=0
    xe=wheelBase+distanceBumperToAxel-engineLocation
    r=wheelDiameter*0.0256+wheelWidth/1000.0*aspectRatio/100.0
    ys=r+1
    xs=wheelBase+distanceBumperToAxel-carLength
    xb=distanceBumperToAxel+wheelBase-carLength/2
    while x<d:
        t=t+dt
        r=wheelDiameter*0.0256+wheelWidth/1000.0*aspectRatio/100.0
        FD=0.5*cod*crossSectionalArea*v**2
        Fd=0.5*DragCurve(spoilerAttackAngle)*areaOfSpoiler*v**2
        FL=0.5*LiftCurve(spoilerAttackAngle)*areaOfSpoiler*v**2
        F2=-(Fd*ys-FL*xs-Fe*xe+h/2*Fd-xb*Fb+r*F)/wheelBase
        F1=Fe+Fb+FD-F2
        gearRatios=[2.97,1.94,1.35,1.00,0.84,0.62]
        hp=0
        for gear in gearRatios:
            rpm=(v/r)*(3.83/1)*gear*(60/2/3.1415)
            if hpCurve(rpm)>hp:
                hp=hpCurve(rpm)
        P=hp*745.70
        at=(F1*mus-Fd-FD)/2/k
        aw=(wheelWidth*Fb-h/2*Fd+xe*Fe-FD*ys+FL*xs-(FD+Fd)*r)/r/2/k
        if v == 0:
            ap=10**8
        else:
            ap=(P-FD-Fd)/2/k
        a=min([aw,at,ap])
        x=x+v*t+0.5*a*t**2
        v=v+a*t
        F=2*k*a+(FD+Fd)
    print(t)
