function[theta,omega] = angle_test(a0,a1)

delta_a1=a1*a0';
theta=acos((trace(delta_a1)-1)/2)*180/pi;
omega=normc(1/(2*sin(theta*pi/180))*[delta_a1(3,2)-delta_a1(2,3);
     delta_a1(1,3)-delta_a1(3,1);delta_a1(2,1)-delta_a1(1,2)] );  %%axis

end