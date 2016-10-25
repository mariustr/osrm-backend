@guidance @merge-segregated
Feature: Merge Segregated Roads

    Background:
        Given the profile "car"
        Given a grid size of 3 meters

    #http://www.openstreetmap.org/#map=18/52.49950/13.33916
    @negative
    Scenario: oneway link road
        Given the node map
            """
            f - - - - - - -_-_e - - - - d
                      ...''
            a - - - b'- - - - - - - - - c
            """

        And the ways
            | nodes | name | oneway |
            | abc   | road | yes    |
            | def   | road | yes    |
            | be    | road | yes    |

        When I route I should get
            | waypoints | route     | intersections                              |
            | a,c       | road,road | true:90,true:60 true:90 false:270;true:270 |
            | d,f       | road,road | true:90,true:60 true:90 false:240;true:270 |

    #http://www.openstreetmap.org/#map=18/52.48337/13.36184
    @negative
    Scenario: Square Area - Same Name as road for in/out
        Given the node map
            """
                                   i
                                   |
                                   |
                                   |
                                   g
                                 /    \
                               /        \
                             /            \
                           /                \
                         /                    \
            a - - - - c                        e - - - - f
                         \                    /
                           \                /
                             \            /
                               \        /
                                 \    /
                                   d
                                   |
                                   |
                                   |
                                   j
            """

        And the ways
            | nodes | name | oneway |
            | ac    | road | no     |
            | ef    | road | no     |
            | cdegc | road | yes    |
            | ig    | top  | no     |
            | jd    | bot  | no     |

        When I route I should get
            | waypoints | route               | intersections | turns |
            | a,f       | road,road,road,road |               | |
