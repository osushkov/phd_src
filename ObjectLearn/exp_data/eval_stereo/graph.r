par(xaxt="n")
par(yaxt="n")

d <- read.table("cat_full.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2] + d[,4] + d[,6]

d <- read.table("cow_full.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,2] + d[,4] + d[,6]

d <- read.table("dog_full.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,2] + d[,4] + d[,6]

d <- read.table("pony_full.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2] + d[,4] + d[,6]

d <- read.table("ship_full.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2] + d[,4] + d[,6]

d <- read.table("skydivers_full.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,2] + d[,4] + d[,6]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/601.0

plot(x, y, type = 'o', ylim = c(0, 1.0), lty=1, pch=0, xlab="", ylab="");
par(new=TRUE)



d <- read.table("cat_full.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,3] + d[,5] + d[,7]

d <- read.table("cow_full.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,3] + d[,5] + d[,7]

d <- read.table("dog_full.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,3] + d[,5] + d[,7]

d <- read.table("pony_full.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3] + d[,5] + d[,7]

d <- read.table("ship_full.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,3] + d[,5] + d[,7]

d <- read.table("skydivers_full.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,3] + d[,5] + d[,7]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/601.0

plot(x, y, type = 'o', ylim = c(0, 1.0), lty=2, pch=1, xlab="", ylab="");
par(new=TRUE)


d <- read.table("cat_full_baseline.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2] + d[,4] + d[,6]

d <- read.table("cow_full_baseline.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,2] + d[,4] + d[,6]

d <- read.table("dog_full_baseline.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,2] + d[,4] + d[,6]

d <- read.table("pony_full_baseline.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2] + d[,4] + d[,6]

d <- read.table("ship_full_baseline.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2] + d[,4] + d[,6]

d <- read.table("skydivers_full_baseline.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,2] + d[,4] + d[,6]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/601.0

plot(x, y, type = 'o', ylim = c(0, 1.0), lty=3, pch=2, xlab="", ylab="");
par(new=TRUE)


par(xaxt="s")
par(yaxt="s")

d <- read.table("cat_full_baseline.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,3] + d[,5] + d[,7]

d <- read.table("cow_full_baseline.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,3] + d[,5] + d[,7]

d <- read.table("dog_full_baseline.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,3] + d[,5] + d[,7]

d <- read.table("pony_full_baseline.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3] + d[,5] + d[,7]

d <- read.table("ship_full_baseline.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,3] + d[,5] + d[,7]

d <- read.table("skydivers_full_baseline.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,3] + d[,5] + d[,7]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/601.0

plot(x, y, type = 'o', ylim = c(0, 1.0), lty=4, pch=3, xlab="Training Frames", ylab="Fraction of Object Features", main="Automatic Segmentation (Translation) vs Manual Segmentation");
par(new=TRUE)

legend("topright", legend = c("Correct Features", "Incorrect Features", "Baseline Correct Features", "Baseline Incorrect Features"), text.width = strwidth("Baseline Incorrect Features"), xjust = 1, yjust = 1, lty = c(1,2,3,4), pch = c(0,1,2,3), bty = "n", cex = 0.8, title = "")
