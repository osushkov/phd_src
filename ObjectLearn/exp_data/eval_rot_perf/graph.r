par(xaxt="n")
par(yaxt="n")


d <- read.table("pony_rot.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2]

d <- read.table("wolf_rot.dat", header=TRUE, sep=",")
wolfx <- d[,1]
wolfy <- d[,2]

x <- ponyx
y <- (ponyy + wolfy)/0.79

plot(x, y, type = 'o', ylim = c(0, 80), lty=1, pch=0, xlab="", ylab="");
par(new=TRUE)


d <- read.table("pony_rot.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3]

d <- read.table("wolf_rot.dat", header=TRUE, sep=",")
wolfx <- d[,1]
wolfy <- d[,3]

x <- ponyx
y <- (ponyy + wolfy)/0.79

plot(x, y, type = 'o', ylim = c(0, 80), lty=2, pch=1, xlab="", ylab="");
par(new=TRUE)



d <- read.table("pony_rot_baseline.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2]

d <- read.table("wolf_rot_baseline.dat", header=TRUE, sep=",")
wolfx <- d[,1]
wolfy <- d[,2]

x <- ponyx
y <- (ponyy + wolfy)/0.79

plot(x, y, type = 'o', ylim = c(0, 80), lty=3, pch=2, xlab="", ylab="");
par(new=TRUE)



par(xaxt="s")
par(yaxt="s")

d <- read.table("pony_rot_baseline.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3]

d <- read.table("wolf_rot_baseline.dat", header=TRUE, sep=",")
wolfx <- d[,1]
wolfy <- d[,3]

x <- ponyx
y <- (ponyy + wolfy)/0.79

plot(x, y, type = 'o', ylim = c(0, 80), lty=4, pch=3, xlab="Training Frames", ylab="Percent of Object Features", main="Automatic Segmentation (Rotation) vs Manual Segmentation");
par(new=TRUE)

legend("topright", legend = c("Correct Features", "Incorrect Features", "Baseline Correct Features", "Baseline Incorrect Features"), text.width = strwidth("Baseline Incorrect Features"), xjust = 1, yjust = 1, lty = c(1,2,3,4), pch = c(0,1,2,3), bty = "n", cex = 0.8, title = "")

