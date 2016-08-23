par(xaxt="n")
par(yaxt="n")

d <- read.table("pony_rot.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2]

d <- read.table("wolf_rot.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2]

x <- catx
y <- (caty + shipy)/79.0

plot(x, y, type = 'o', ylim = c(0, 0.7), lty=3, pch=3, xlab="", ylab="");
par(new=TRUE)


par(xaxt="s")
par(yaxt="s")

d <- read.table("pony_rot_baseline.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2]

d <- read.table("wolf_rot_baseline.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2]

x <- catx
y <- (caty + shipy)/79.0

plot(x, y, type = 'o', ylim = c(0, 0.7), lty=3, pch=3, xlab="", ylab="");
par(new=TRUE)


#legend("topright", legend = c("SVM-pairwise"), text.width = strwidth("SVM-pairwise"), xjust = 1, yjust = 1, lty = c(1), pch = c(1), bty = "n", cex = 0.8, title = "")
