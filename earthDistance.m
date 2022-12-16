function dist = earthDistance(pair1,pair2)
  %note longitude comes first in each pair
  R = 6371*10^3; %radius of earth
  %need radians for trig
  lat1 = pair1(2)*pi()/180;
  lat2 = pair2(2)*pi()/180;
  lon1 = pair1(1)*pi()/180;
  lon2 = pair2(1)*pi()/180;
  delLat = lat2-lat1;
  delLon = lon2-lon1;
  a = sin(delLat/2)^2 + cos(lat1)*cos(lat2)*sin(delLon/2)^2;
  c = 2*atan2(sqrt(a),sqrt(1-a));
  dist = R*c;
  
endfunction
