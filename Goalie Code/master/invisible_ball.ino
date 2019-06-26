boolean farAway;
int lastxPos;
int lastyPos;

void prepInvBall(){
  if(ballDist > 350) farAway = true;
  else farAway = false;
  lastxPos = xPos;
  lastyPos = yPos;
}


void actInvBall(){
  
}
//if the ball is far away and the ball disappears -> go to center of the goal region.

//if the ball is near and the ball disappears -> go to that edge of the goal.
