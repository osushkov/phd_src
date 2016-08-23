
d <- read.table("total.dat", header=TRUE, sep=",")

# create a two row matrix with x and y
height <- rbind(d[,1], d[,2], d[,3], d[,4])
height = height/3.0 # 3 objects tested, so div by 3
height = ceiling(height)

# Use height and set 'beside = TRUE' to get pairs
# save the bar midpoints in 'mp'
# Set the bar pair labels to A:D
mp <- barplot(height, beside=TRUE, ylim = c(0, 600), ylab="Number of Features Detected", main="Feature Scale Filtering", cex.names=0.8, legend=c("True arm features", "True object features", "False arm features", "False object features"))
text(mp, height, labels = format(height, 4),pos = 3, cex = 0.7)

mtext(1, at = colMeans(mp), text = c("Filtering Enabled", "Filtering Disabled"), line = 2)
