d <- read.table("eval_ponyclean_full.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
plot(x, y, type = 'o', ylim = c(0, 45), lty=1, pch=1, xlab="Learning Frames", ylab="Detected Features");

d <- read.table("eval_ponyclean_ntb.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
par(new=TRUE)
plot(x, y, type = 'o', ylim = c(0, 45), lty=2, pch=2, xlab="Learning Frames", ylab="Detected Features")


#legend("topright", legend = c("SVM-pairwise"), text.width = strwidth("SVM-pairwise"), xjust = 1, yjust = 1, lty = c(1), pch = c(1), bty = "n", cex = 0.8, title = "")
