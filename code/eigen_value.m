global A = csvread('csv_matter.csv'); #do not change this line
global n;
global filterax;
global filteray;
global filteraz;
global filtergx;
global filtergy;
global filtergz;
global pitch;
global roll;
global B;
global gx;
global gy;
global gz;

################################################
#######Declare your global variables here#######
################################################


function read_accel(axl,axh,ayl,ayh,azl,azh)  
  global n;
  
  
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####
  #################################################
  axh=dec2bin(axh,8);
  axl=dec2bin(axl,8);
  ayh=dec2bin(ayh,8);
  ayl=dec2bin(ayl,8);
  azh=dec2bin(azh,8);
  azl=dec2bin(azl,8);
  axx=[axh axl];
  axx=bin2dec(axx);
  if(axx>32767)
    axx=axx-65536;
  endif
  ayy=[ayh ayl];
  ayy=bin2dec(ayy);
  if(ayy>32767)
    ayy=ayy-65536;
  endif
  azz=[azh azl];
  azz=bin2dec(azz);
  if(azz>32767)
    azz=azz-65536;
  endif
  ax(n)=axx/16384.0;
  ay(n)=ayy/16384.0;
  az(n)=azz/16384.0;
  f_cut=5;
  
  
  

  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  ####################################################
  lowpassfilter(ax,ay,az,f_cut);

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  global n;
  global gx;
  global gy;
  global gz;
  
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  #################################################
  
  gxh=dec2bin(gxh,8);
  gxl=dec2bin(gxl,8);
  gyh=dec2bin(gyh,8);
  gyl=dec2bin(gyl,8);
  gzh=dec2bin(gzh,8);
  gzl=dec2bin(gzl,8);
  gxx=[gxh gxl];
  gxx=bin2dec(gxx);
  if(gxx>32767)
    gxx=gxx-65536;
  endif
  
  gyy=[gyh gyl];
  gyy=bin2dec(gyy);
  if(gyy>32767)
    gyy = gyy-65536;
  endif
  
  gzz=[gzh gzl];
  gzz=bin2dec(gzz);
  if(gzz>32767)
    gzz=gzz-65536;
  endif
  gx(n)=gxx/131.0;
  gy(n)=gyy/131.0;
  gz(n)=gzz/131.0;
   f_cut=5;
  #####################################################
  # Call function highpassfilter(ax,ay,az,f_cut) here #
  #####################################################;
    highpassfilter(gx,gy,gz,f_cut);
endfunction



function lowpassfilter(ax,ay,az,f_cut)
  global n;
  global filterax;
  global filteray;
  global filteraz;
  
  

  dT = 0.01;  #time in seconds
  Tau=1/(2*pi*f_cut) ;
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  if (n==1)
    {
    filterax(n)=(1-alpha)*ax(n) ;
    filteray(n)=(1-alpha)*ay(n);
    filteraz(n)=(1-alpha)*az(n);
    }
    
  endif
  
  if(n>1)
    {
    filterax(n)=(1-alpha)*ax(n) + alpha*(filterax(n-1));
    filteray(n)=(1-alpha)*ay(n) + alpha*(filteray(n-1));
    filteraz(n)=(1-alpha)*az(n)+  alpha*(filteraz(n-1));
    }
  endif
  
 
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  global n;
  global filtergx;
  global filtergy;
  global filtergz;
  
  
  dT = 0.01;  #time in seconds
  Tau=1/(2*pi*f_cut) ;
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
   if(n==1)
  {
    filtergx(n) = (1-alpha)*(gx(n));
    filtergy(n) = (1-alpha)*(gy(n));   
    filtergz(n) = (1-alpha)*(gz(n));  
  }
  endif
  if(n>1)
  {
    filtergx(n) = (1-alpha)*filtergx(n-1) + (1-alpha)*((gx(n))-((gx(n-1))));
    filtergy(n) = (1-alpha)*filtergy(n-1) + (1-alpha)*((gy(n))-((gy(n-1))));
    filtergz(n) = (1-alpha)*filtergz(n-1) + (1-alpha)*((gz(n))-((gz(n-1))));
    
    }
  endif
  
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)
  global n;
 
  global pitch;

  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  ##############################################
##  alpha =0.03 as given by instructor in piazza
  
  dT = 0.01;
  acc_pitch=(180/pi)*(atan(ay(n)/sqrt(az(n)^2)));
  if(n==1)
    {
    pitch(n)=(.97)*(gx(n)*dT)+(.03)*(acc_pitch);
    
    }
  endif
  if(n>1)
    {      
    pitch(n)=(0.97)*(pitch(n-1)+gx(n)*dT)+(0.03)*(acc_pitch);
    
    }
  endif
  

endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)
  global n;
  global roll;
 
  
  #######################\#######################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  ##############################################
  dT = 0.01;
  acc_roll =(180/pi)*(atan(ax(n)/sqrt(az(n)^2)));
  if(n==1)
    {
    
    roll(n)=(.97)*(+gy(n)*dT)+(.03)*(acc_roll);
    }
  endif
  if(n>1)
    {      
    
    roll(n)=(0.97)*(roll(n-1)+gy(n)*dT)+(0.03)*(acc_roll);
    }
  endif

endfunction 

function execute_code
  global filterax;
  global filteray;
  global filteraz;
  global filtergx;
  global filtergy;
  global filtergz;
  global pitch;
  global roll;
  global A;
  global B;
  global n;
  B=zeros(8000,2);
  

  for n = 1:rows(A) #do not change this line
    axh=A(n,1);
    axl=A(n,2);
    ayh=A(n,3);
    ayl=A(n,4);
    azh=A(n,5);
    azl=A(n,6);
    gxh=A(n,7);
    gxl=A(n,8);
    gyh=A(n,9);
    gyl=A(n,10);
    gzh=A(n,11);
    gzl=A(n,12);
    
    read_accel(axl,axh,ayl,ayh,azl,azh);
    read_gyro(gxl,gxh,gyl,gyh,gzl,gzh);
    comp_filter_pitch(filterax,filteray,filteraz,filtergx,filtergy,filtergz);
    comp_filter_roll(filterax,filteray,filteraz,filtergx,filtergy,filtergz);
     
    
    B(n,1)=pitch(n);
    B(n,2)=roll(n);
    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    ###############################################
    
  endfor
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
