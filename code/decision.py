import numpy as np
import time
import math

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'start':
            nav_left = np.count_nonzero( Rover.nav_angles > 0 )
            nav_total = len( Rover.nav_angles )
            rate = nav_left / nav_total
            if rate > 0.35 and nav_total > 1500:
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                print( '[Start] nav_left = {:.3f}, [{:4d} + {:4d} = {:4d}]. vel = {:.2f}'.format( rate, \
                    nav_left, nav_total - nav_left, nav_total, Rover.vel ) )
            else:
                Rover.mode = 'forward'
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15

                # nav_left = np.count_nonzero( Rover.nav_angles > 0 )
                # nav_total = len( Rover.nav_angles )
                # rate = nav_left / nav_total
                # if rate > 0.35 and nav_total > 1500:
                #     Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                # else:
                ### =============== My code here =============== ###
                check_stuck( Rover )

                if Rover.stuck:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                    Rover.stuck_yaw = ( Rover.yaw - 15 + 360 ) % 360
                    Rover.mode = 'stuck'
                else:
                    update_steer( Rover )
                ### =============== My code here =============== ###
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            if math.fabs( ( ( Rover.stuck_yaw + 360 ) % 360 ) - Rover.yaw ) < 5:
                print( '[Stuck] Get there.' )
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'stuck_forward'

                    Rover.stuck = False
                    Rover.stuck_begin = None
                else:
                    Rover.stuck_yaw = ( Rover.yaw - 15 + 360 ) % 360
            else:
                print( '[Stuck] Turning... {:.1f} - {:.1f} = {:.1f}'.format( \
                    Rover.stuck_yaw, Rover.yaw, math.fabs( ( ( Rover.stuck_yaw + 360 ) % 360 ) - Rover.yaw ) ) )
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15
        elif Rover.mode == 'stuck_forward':
            check_stuck( Rover )
            if Rover.stuck:
                print( '[stuck_forward] Stuck again.' )
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15
                Rover.stuck_yaw = ( Rover.yaw - 15 + 360 ) % 360
                Rover.mode = 'stuck'
            elif Rover.vel < 0.5:
                print( '[stuck_forward] Try to forward.' )
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            else:
                print( '[stuck_forward] Get out of stuck.' )
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

def update_steer( Rover ):
    # Put new steer into buff
    nav_left = np.count_nonzero( Rover.nav_angles > 0 )
    nav_total = len( Rover.nav_angles )

    rate = nav_left / nav_total
    ref_rate = 0.20

    steer = np.clip( ( rate - ref_rate ) / ref_rate * 15, -15, 15 )

    delay = 0.1 # 50ms
    now = time.time()
    Rover.steer_buff.append( [ now + delay, steer ] )

    print( 'nav_left = {:.3f}, [{:4d} + {:4d} = {:4d}] Go {}. vel = {:.2f}'.format( rate, \
            nav_left, nav_total - nav_left, nav_total, 'right' if rate < ref_rate else 'left', \
            Rover.vel ) )

     # Retrieve latest steer
    k = -1
    for i in range( len( Rover.steer_buff ) ):
        if Rover.steer_buff[ i ][ 0 ] <= now:
            k = i
        else:
            break
    if k >= 0:
        Rover.steer = Rover.steer_buff[ k ][ 1 ]
        Rover.steer_buff = Rover.steer_buff[ k + 1 : ]

    return Rover

def check_stuck( Rover ):
    # print( '[check_stuck] mode = {}, throttle = {}, stuck = {}, begin = {}'.format( \
    #         Rover.mode, Rover.throttle, Rover.stuck, Rover.stuck_begin ) )
    # Do nothing in stop mode.
    if Rover.mode == 'stop' or Rover.throttle == 0:
        return Rover

    thresh_vel = 0.1
    thresh_time = 0.5

    if Rover.vel > thresh_vel:
        Rover.stuck_begin = None
        Rover.stuck = False
        return Rover

    if Rover.stuck_begin is None:
        Rover.stuck_begin = time.time()
    elif time.time() - Rover.stuck_begin >= thresh_time:
        Rover.stuck = True
