
dir=$1
angle=$2
cdir=`pwd`

cd ${dir}

a=1
for file in *.pcd; do
  new=$(printf "%01d_"${angle}"deg_raw.pcd" "$a") #01 pad to length of 1
  mv -i -- "$file" "$new"
  let a=a+1
done

cd ${cdir}