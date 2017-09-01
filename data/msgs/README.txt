**********************************************
******** HOW THE MSGS ARE ORGANIZED **********
**********************************************

BetterPushback voice messages are placed into this directory and are
organized by a fixed directory naming convention. The set of available
voice message variants is NOT hard-coded into BP. Instead, this directory
is enumerated at runtime to pick a suitable voice set.

A voice set directory name must be one of the following:

1) ISO 639-1 code, followed by underscore character, followed by
   ISO-3166-1 alpha 2 code. Example: "pt_BR". Denotes a language pack
   for a specific language dialect spoken in a particular country.

2) ISO 639-1 only. Example: "ru". Denotes a language pack used as a
   generic language fallback if a country-specific pack isn't available.

3) "en", followed by an underscore character, followed by ISO-3166-1
   alpha 2 code. Example: "en_BR". This denotes a localized accented
   English pack specific to one country.

4) "en", followed by a dash character, followed by ISO-639-1 code.
   Example: "en-pt". Denotes a localized accented English pack used as a
   fallback when the local country language differs from English, but a
   local country pack following rule #3 above isn't available.

Any of the above rules may be suffixed by an optional variant name in
parentheses. This is used for creating multiple variants of the same
voice set. An example would be 'en(jonfly)', which is a variant of US
English using the voice of Twitch streamer JonFly.

Anything not conforming to these naming conventions is ignored. The exact
method for selecting a language set is described in src/msg.c in the
msg_init function.
