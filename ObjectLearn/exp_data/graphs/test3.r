par(xaxt="n")
par(yaxt="n")

d <- read.table("eval_catclean_full.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2]

d <- read.table("eval_cowclean_full.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,2]

d <- read.table("eval_dogclean_full.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,2]

d <- read.table("eval_ponyclean_full.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2]

d <- read.table("eval_shipclean_full.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2]

d <- read.table("eval_skydiversclean_full.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,2]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/384.0


plot(x, y, type = 'o', ylim = c(0, 0.7), lty=1, pch=1, xlab="", ylab="");
par(new=TRUE)

d <- read.table("eval_catclean_ntb.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,2]

d <- read.table("eval_cowclean_ntb.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,2]

d <- read.table("eval_dogclean_ntb.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,2]

d <- read.table("eval_ponyclean_ntb.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,2]

d <- read.table("eval_shipclean_ntb.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,2]

d <- read.table("eval_skydiversclean_ntb.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,2]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/384.0

plot(x, y, type = 'o', ylim = c(0, 0.7), lty=2, pch=2, xlab="", ylab="");
par(new=TRUE)

d <- read.table("eval_catclean_full.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,3]

d <- read.table("eval_cowclean_full.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,3]

d <- read.table("eval_dogclean_full.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,3]

d <- read.table("eval_ponyclean_full.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3]

d <- read.table("eval_shipclean_full.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,3]

d <- read.table("eval_skydiversclean_full.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,3]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/384.0

plot(x, y, type = 'o', ylim = c(0, 0.7), lty=3, pch=3, xlab="", ylab="");
par(new=TRUE)


d <- read.table("eval_catclean_ntb.dat", header=TRUE, sep=",")
catx <- d[,1]
caty <- d[,3]

d <- read.table("eval_cowclean_ntb.dat", header=TRUE, sep=",")
cowx <- d[,1]
cowy <- d[,3]

d <- read.table("eval_dogclean_ntb.dat", header=TRUE, sep=",")
dogx <- d[,1]
dogy <- d[,3]

d <- read.table("eval_ponyclean_ntb.dat", header=TRUE, sep=",")
ponyx <- d[,1]
ponyy <- d[,3]

d <- read.table("eval_shipclean_ntb.dat", header=TRUE, sep=",")
shipx <- d[,1]
shipy <- d[,3]

d <- read.table("eval_skydiversclean_ntb.dat", header=TRUE, sep=",")
skydiversx <- d[,1]
skydiversy <- d[,3]

x <- ponyx
y <- (caty + cowy + dogy + ponyy + shipy + skydiversy)/384.0

par(xaxt="s")
par(yaxt="s")

plot(x, y, type = 'o', ylim = c(0, 0.7), lty=4, pch=4, xlab="Learning Frames", ylab="Detected Features");




#legend("topright", legend = c("SVM-pairwise"), text.width = strwidth("SVM-pairwise"), xjust = 1, yjust = 1, lty = c(1), pch = c(1), bty = "n", cex = 0.8, title = "")
