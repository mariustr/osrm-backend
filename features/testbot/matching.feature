@match @testbot
Feature: Basic Map Matching

    Background:
        Given the profile "testbot"
        Given a grid size of 10 meters
        Given the extract extra arguments "--generate-edge-lookup"
        Given the query options
            | geometries | geojson |

    Scenario: Testbot - Map matching with outlier that has no candidate
        Given a grid size of 10 meters
        Given the node map
            """
            a b c d
                1
            """

        And the ways
            | nodes | oneway |
            | abcd  | no     |

        When I match I should get
            | trace | timestamps | matchings |
            | ab1d  | 0 1 2 3    | abcd      |

    Scenario: Testbot - Map matching with trace splitting
        Given the node map
            """
            a b c d
                e
            """

        And the ways
            | nodes | oneway |
            | abcd  | no     |

        When I match I should get
            | trace | timestamps | matchings |
            | abcd  | 0 1 62 63  | ab,cd     |

    Scenario: Testbot - Map matching with core factor
        Given the contract extra arguments "--core 0.8"
        Given the node map
            """
            a b c d
                e
            """

        And the ways
            | nodes | oneway |
            | abcd  | no     |

        When I match I should get
            | trace | timestamps | matchings |
            | abcd  | 0 1 2 3    | abcd      |

    Scenario: Testbot - Map matching with small distortion
        Given the node map
            """
            a b c d e
              f



              h     k
            """

        # The second way does not need to be a oneway
        # but the grid spacing triggers the uturn
        # detection on f
        And the ways
            | nodes | oneway |
            | abcde | no     |
            | bfhke | yes    |

        When I match I should get
            | trace  | matchings |
            | afcde  | abcde     |

    Scenario: Testbot - Map matching with oneways
        Given a grid size of 10 meters
        Given the node map
            """
            a b c d
            e f g h
            """

        And the ways
            | nodes | oneway |
            | abcd  | yes    |
            | hgfe  | yes    |

        When I match I should get
            | trace | matchings |
            | dcba  | hgfe      |

	Scenario: Testbot - Matching with oneway streets -- returns the correct trace but not broken up. also tracepoints names are weird
        Given a grid size of 10 meters
        Given the node map
            """
            a b c d
            e f g h
            """

        And the ways
            | nodes | oneway |
            | ab    | yes    |
            | bc    | yes    |
            | cd    | yes    |
            | hg    | yes    |
            | gf    | yes    |
            | fe    | yes    |

        When I match I should get
            | trace | matchings |
            | dcba  | hgfe  |
            | efgh  | abcd  |

    Scenario: Testbot - Duration details -- returns incomplete matching even within tracepoints. matchings also obviously incomplete so not possible to fill in details in annotaion table. also, current table only has 3 properties, but new API returns 4 properties  
        Given the query options
            | annotations | true    |

        Given the node map
            """
            a b c d e   g h
                i
            """

        And the ways
            | nodes    | oneway |
            | abcdegh  | no     |
            | ci       | no     |

        And the speed file
        """
        1,2,36
        """

        And the contract extra arguments "--segment-speed-file {speeds_file}"

        When I match I should get
            | trace | matchings | annotation                                                                                     |
            | abeh  | abcedgh   | 1:9.897633:1,0:0:0,1:10.008842:0,1:10.008842:0,1:10.008842:0,0:0:0,2:20.017685:0,1:10.008842:0 |
            | abci  | abc,ci    | 1:9.897633:1,0:0:0,1:10.008842:0,0:0.111209:0,1:10.010367:0                                    |

        # The following is the same as the above, but separated for readability (line length)
        When I match I should get
            | trace | matchings | OSM IDs               |
            | abeh  | abcedgh   | 1,2,3,2,3,4,5,4,5,6,7 |
            | abci  | abc,ci    | 1,2,3,2,3,8,3,8       |

    Scenario: Testbot - Geometry details -- precision problems
        Given the query options
            | overview   | full     |
            | geometries | geojson  |

        Given the node map
            """
            a b c
              d
            """

        And the ways
            | nodes | oneway |
            | abc   | no     |
            | bd    | no     |

        When I match I should get
            | trace | matchings | geometry                                |
            | abd   | abd       | 1,1,1.00009,1,1.00009,1,1.00009,0.99991 |

    Scenario: Testbot - Speed greater than speed threshhold, should split -- returns trace as abcd but should be split into ab,cd
        Given a grid size of 10 meters
        Given the node map
            """
            a b              c d
                             e
            """

        And the ways
            | nodes | oneway |
            | abcd  | no     |

        When I match I should get
            | trace | timestamps | matchings |
            | abcd  | 0 1 2 3    | ab,cd     |

    Scenario: Testbot - Speed less than speed threshhold, should not split  
        Given a grid size of 10 meters
        Given the node map
            """
            a b c d
                e
            """

        And the ways
            | nodes | oneway |
            | abcd  | no     |

        When I match I should get
            | trace | timestamps | matchings |
            | abcd  | 0 1 2 3    | abcd      |
