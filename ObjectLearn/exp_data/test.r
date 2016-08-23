
d <- read.table("match_scatterplot.dat", sep=",")


# Use height and set 'beside = TRUE' to get pairs
# save the bar midpoints in 'mp'
# Set the bar pair labels to A:D
mp <- plot(d, ylim=c(0.0, 4.0), xlim=c(0, 35), xlab="", ylab="", main="Edge Distance and Scale of Matched and Unmatched Features", col="green", pch=c(1))

par(new=TRUE)

d <- read.table("nonmatch_scatterplot.dat", sep=",")


# Use height and set 'beside = TRUE' to get pairs
# save the bar midpoints in 'mp'
# Set the bar pair labels to A:D
mp <- plot(d, ylim=c(0.0, 4.0), xlim=c(0, 35), xlab="Border Distance (pixels)", ylab="Feature Scale", col="red", pch=c(2))

legend("bottomright", legend = c("Detected Features", "Undetected Features"), text.width = strwidth("Undetected Features"), xjust = 1, yjust = 1, lty = c(0, 0), pch = c(1, 2), bty = "n", cex = 0.8, title = "", col=c("green", "red"))
