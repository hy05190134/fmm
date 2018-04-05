### UBODT serialization with boost library

Run `ubodt2binary` with

    make
    ubodt2binary ubodt.txt ubodt.binary


### Compression test

The statistics are collected from converting a large UBODT file in CSV format (**5 million**) rows. 

| Format          | size |
|-----------------|------|
| CSV             | 193M |
| hdf5 (pandas)   | 172M |
| Pickle (pandas) | 115M |
| Binary (boost)  | 115M |

