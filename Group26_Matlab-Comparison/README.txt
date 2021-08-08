code to run the 'Dynamic numerical.m':

    ode     = cheetah02( ell );
    solver = RK4(); % initialize solver
    solver.setODE(ode1);       % Attach ode to the solver
    T = 1;
    h    = 0.0001;
    tt   = 0:h:Tmax;
    theta10=-pi/4;theta20=-2.912407310;theta1__dot0=0;theta2__dot0=0;y10=1;y1__dot0=0;
    ini=[theta10,theta20,y10,theta1__dot0,theta2__dot0,y1__dot0];
    sol = solver.advance( tt, ini );
    T_M1=-50*(sol1(1,:)+pi/4);figure(1);plot(tt,T_M1);%T_M1
    s = sqrt(2*0.26*0.42*cos(sol1(1,:)-sol1(2,:))+0.26^2+0.42^2);figure(2);plot(tt,s); %s(t)
    figure(3);plot(tt,sol(1,:));%y1