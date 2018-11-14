#tests.py

#TEST_1
#position = [0,-math.pi/2,math.pi/4,-math.pi/2,0,0,1]

#Test2
#position = [math.pi/4,-math.pi/2,math.pi/4,-math.pi/2,0,0,0]

#Test3
# position = [0,-math.pi/2,math.pi/4,-math.pi/2,0,0,1]
# ur5.setConfig(position)
# time.sleep(0.1)
# position = [0,-math.pi/2,0,-math.pi/2,0,0,0]

#Test4
# position = [math.pi/4,-math.pi/2,math.pi/4,-math.pi/2,0,0,0]
# ur5.setConfig(position)
# time.sleep(0.1)
# position = [0,-math.pi/2,0,-math.pi/2,0,0,0]
# ur5.setConfig(position)
# time.sleep(2)

#Test5
# start_time=time.time()
# while time.time()-start_time < 15:
#     t=time.time()-start_time
#     q1=0.7*math.sin(t)
#     q3=0.7*math.sin(t)
#     q7=abs(math.sin(0.5*t))
#     position = [q1,-math.pi/2,q3,-math.pi/2,0,0,q7]
#     ur5.setConfig(position)
#     time.sleep(0.005)

#Test6
# start_time=time.time()
# while time.time()-start_time < 15:
#     t=time.time()-start_time
#     q1=0.7*math.cos(2*t)
#     q3=0.7*math.cos(2*t)
#     q7=abs(math.cos(t))
#     position = [q1,-math.pi/2,q3,-math.pi/2,0,0,q7]
#     ur5.setConfig(position)
#     time.sleep(0.005)

#Test7
# velocity = [0,0,math.pi/4,0,0,0,1]
# ur5.setVelocity(velocity)
# time.sleep(1)

#Test8
# velocity = [math.pi/4,0,math.pi/4,0,0,math.pi/4,0]
# ur5.setVelocity(velocity)
# time.sleep(1)

#Test9
# velocity = [math.pi/2,0,math.pi/2,0,0,math.pi/2,1]
# ur5.setVelocity(velocity)
# time.sleep(0.1)
# velocity = [0,0,0,0,0,0,0]
# ur5.setVelocity(velocity)
# time.sleep(0.5)


#Test10
# start_time=time.time()
# while time.time()-start_time < 15:
#     t=time.time()-start_time
#     v1=1*math.sin(2*t)
#     v3=1*math.sin(2*t)
#     v7=0.5*math.sin(t)
#     velocity = [v1,0,v3,0,0,0,v7]
#     ur5.setVelocity(velocity)
#     time.sleep(0.005)    