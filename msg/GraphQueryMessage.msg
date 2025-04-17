# Languages in which graph queries can be encoded.
uint8 LANG_FOL=0 # Prolog-like syntax, @see ....

# Configure the language in which the graph query is encoded.
string lang # Default: LANG_FOL

# A string encoding a graph query that is to be evaluated
# wrt. the knowledge graph selected by the modality configured
# in this message.
# The query must by valid expression in the language configured
# via the "lang" field.
string queryString

# Determines the modal frame in which the triple is assumed to be true.
ModalFrame frame
