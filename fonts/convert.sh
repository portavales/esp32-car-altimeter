
rm ../fonts.h
for vlw_filename in *.vlw; do
    fontname=${vlw_filename%.*}
    fontname=$(echo $fontname | sed 's/-/_/g')
    h_filename=${fontname}.h
    rm $h_filename
    echo "Converting ${vlw_filename} to ${h_filename}"
    echo "#pragma once" >> $h_filename
    echo "const uint8_t  ${fontname}[] PROGMEM = { " >> $h_filename
    hexdump -ve '1/1 "0x%02X,"' $vlw_filename >> $h_filename
    echo "};" >> $h_filename

    echo "#include \"fonts/${h_filename}\"" >> ../fonts.h
done
