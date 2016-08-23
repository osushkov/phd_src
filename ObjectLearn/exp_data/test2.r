
d <- read.table("skydiversclean_distribution.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
total = y

#plot(x, y, type = 'o', xlim = c(0, 30), ylim = c(0, 50), lty=1, pch=1);

d <- read.table("skydiverscluttered_distribution.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
total = total + y

#par(new=TRUE)
#plot(x, y, type = 'o', xlim = c(0, 30), ylim = c(0, 50), lty=2, pch=2)

d <- read.table("cupclean_distribution.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
total = total + y
#par(new=TRUE)
#plot(x, y, type = 'o', xlim = c(0, 30), ylim = c(0, 50), lty=3, pch=3)

d <- read.table("cupcluttered_distribution.dat", header=TRUE, sep=",")
x <- d[,1]
y <- d[,2]
total = total + y;
total = total/4.0
#total[1] + total[2] + total[3] + total[4] + total[5] + total[6] + total[7] + total[8] + total[9] + total[10] + total[11] + total[12] + total[13] + total[14]

#par(new=TRUE)
plot(x, total, type = 'l', xlim = c(0, 32), lty=3, main="SIFT Scale Distribution", xlab="Feature Scale", ylab="Percentage")


#legend("topright", legend = c("SVM-pairwise"), text.width = strwidth("SVM-pairwise"), xjust = 1, yjust = 1, lty = c(1), pch = c(1), bty = "n", cex = 0.8, title = "")
