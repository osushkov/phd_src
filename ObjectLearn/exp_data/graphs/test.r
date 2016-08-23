
d <- read.table("all_data.dat", header=TRUE, sep=",")

# create a two row matrix with x and y
height <- rbind(d[,2], d[,3], d[,4], d[,5])

# Use height and set 'beside = TRUE' to get pairs
# save the bar midpoints in 'mp'
# Set the bar pair labels to A:D
mp <- barplot(height, beside = TRUE, ylim = c(0, 25), names.arg = LETTERS[1:4], main="Weeee")
text(mp, height, labels = format(height, 4),pos = 3, cex = .65)

