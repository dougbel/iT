looks <-1000

#these are the mapped values 
x_min = 1
x_max = 100

#generate the X-axis values
x<-numeric(looks)
stepw<-(x_max-x_min)/looks
for (i in 0:looks){
  x[i+1] <-x_min+stepw*i
}

#defined fucntion for mapping
y = 1/x


#plotting values mapped and their output
plot(x,y, type="l", col="green", lwd=5, xlab="time", ylab="concentration", main="Values mapped")