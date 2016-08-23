
d <- read.table("total.dat", header=TRUE, sep=",")

# create a two row matrix with x and y
height <- rbind(d[,1], d[,2], d[,3], d[,4], d[,5])
height = height/3.0
height = ceiling(height)

# Use height and set 'beside = TRUE' to get pairs
# save the bar midpoints in 'mp'
# Set the bar pair labels to A:D
mp <- barplot(height, beside=TRUE, ylim = c(0, 700), ylab="Number of Features Detected", main="Arm Cross-Correlation", cex.names=0.8, legend=c("True arm features", "True object features", "False arm features", "False object features", "Correlation corrections"))
text(mp, height, labels = format(height, 5) ,pos = 3, cex = 0.7)

mtext(1, at = colMeans(mp), text = c("Correlation Enabled", "Correlation Disabled"), line = 2)
