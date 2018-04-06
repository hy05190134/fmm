### More efficient MM IO

Install 

    sudo apt install spatialite-bin

### Spatialite queries in SQL

```
    $ spatialite result.sqlite
    $ .tables
    $ pragma table_info(result);

    0|ogc_fid|INTEGER|0||1
    1|id|INTEGER|0||0
    2|o_path|VARCHAR(255)|0||0
    3|c_path|VARCHAR(255)|0||0
    4|GEOMETRY|LINESTRING|0||0

    $ select max(length(o_path)) from result;

    max(length(o_path))
    597

    # Get the row with maximum value of o_path (id of 21630)
    $ SELECT * FROM result ORDER BY length(o_path) DESC LIMIT 1;

    # Check geometry of the result
    $ SELECT *,ST_AsText(GEOMETRY) FROM result where ogc_fid = 100
```


#### References

1. Spatialite quick start https://live.osgeo.org/en/quickstart/spatialite_quickstart.html
2. Spatialite C API example http://www.gaia-gis.it/gaia-sins/splite-doxy-4.3.0/examples.html
