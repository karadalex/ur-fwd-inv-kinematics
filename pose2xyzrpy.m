function r = pose2xyzrpy(M)
    x = M(1,4);
    y = M(2,4);
    z = M(3,4);
    roll = atan2(M(2,1),M(1,1));
    pitch = atan2(-M(3,1), sqrt(M(1,1)^2 + M(2,1)^2));
    yaw = atan2(M(3,2),M(3,3));
    r = [x,y,z,roll,pitch,yaw];
end

