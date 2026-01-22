from canmatrix import convert

dbc_file_name = 'MX11_1.5_ADD_ChassisFusionCANFD_240601.dbc'
convert.convert(
    dbc_file_name,
    dbc_file_name.replace('dbc','csv')
)
