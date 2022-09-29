# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Copyright 2022 Saso Kiselkov
#
# This file is environment-specific and must define the following
# two variables:
#	DEVELOPER_USERNAME := "apple_ID@whatever.com"
#	DEVELOPER_PASSWORD := "@keychain:ALTOOL_KEYCHAIN_ENTRY_NAME"
# These will be passed to the notarization tool (altool) to authenticate
# the notarization request to Apple.

TOPDIR := notarize

include $(TOPDIR)/user.make

# To codesign and notarize the product, simply invoke:
#
#	make notarize
#

.PHONY: notarize codesign

SIGNING_CERTIFICATE := $(shell security find-certificate -Z -c \
    "Developer ID Application:" | grep "SHA-1" | awk 'NF { print $$NF }')
TEAM_ID := $(shell security find-certificate -c "Developer ID Application:" | \
    grep "alis" | awk 'NF { print $$NF }' | tr -d \(\)\")
XP_PLUGIN := mac_x64/BetterPushback.xpl
ZIPNAME := $(shell mktemp -u /tmp/bp.XXXXXX).zip
UPLOAD_INFO_PLIST := $(TOPDIR)/UploadInfo.plist
REQUEST_INFO_PLIST := $(TOPDIR)/RequestInfo.plist
AUDIT_INFO_JSON := $(TOPDIR)/AuditInfo.json
EXPORT_OPTS := $(TOPDIR)/ExportOptions.plist

ifneq ($(VERBOSE),1)
	VERB := @
endif

CODESIGN := /usr/bin/codesign
ALTOOL := /usr/bin/xcrun altool
STAPLER := /usr/bin/xcrun stapler
PLUTIL := /usr/bin/plutil
PLIST_BUDDY := /usr/libexec/PlistBuddy
CURL := /usr/bin/curl
ZIP := /usr/bin/zip

define wait_for_notarization
	$(VERB) while true; do \
		sleep 60 ;\
		$(ALTOOL) --notarization-info \
		    "`$(PLIST_BUDDY) -c \
		    "Print :notarization-upload:RequestUUID"\
		     $(UPLOAD_INFO_PLIST)`" -u $(DEVELOPER_USERNAME) \
		     -p $(DEVELOPER_PASSWORD) --output-format xml > \
		     $(REQUEST_INFO_PLIST) ; \
		if [ "`$(PLIST_BUDDY) -c \
		    "Print :notarization-info:Status" \
		    $(REQUEST_INFO_PLIST)`" != "in progress" ]; then \
			break ;\
		fi ;\
		echo "In progress, will check again in 60 seconds...";\
	done
endef

codesign :
	@echo "Codesigning product..."
	$(VERB) $(CODESIGN) -v $(XP_PLUGIN) 2> /dev/null || \
	    $(CODESIGN) -s $(SIGNING_CERTIFICATE) \
	    --options runtime --timestamp $(XP_PLUGIN)

notarize : codesign
	@echo "Replacing export options..."
	$(VERB) cp $(EXPORT_OPTS).orig $(EXPORT_OPTS)
	$(VERB) $(PLUTIL) -replace signingCertificate -string \
	    $(SIGNING_CERTIFICATE) $(EXPORT_OPTS)
	$(VERB) $(PLUTIL) -replace teamID -string $(TEAM_ID) $(EXPORT_OPTS)
	@echo "Packaging into ZIP..."
	$(VERB) $(ZIP) $(ZIPNAME) $(XP_PLUGIN)
	@echo "Uploading for notarization..."
	$(VERB) $(ALTOOL) --notarize-app --primary-bundle-id \
	    "x-plane.BetterPushback" -u $(DEVELOPER_USERNAME) \
	    -p $(DEVELOPER_PASSWORD) -f $(ZIPNAME) \
	    --output-format xml > $(UPLOAD_INFO_PLIST)
	@echo "Waiting for notarization (this can take several minutes)..."
	$(VERB) $(ALTOOL) --notarization-info `$(PLIST_BUDDY) \
	    -c "Print :notarization-upload:RequestUUID" $(UPLOAD_INFO_PLIST)` \
	    -u $(DEVELOPER_USERNAME) -p $(DEVELOPER_PASSWORD) \
	    --output-format xml > $(REQUEST_INFO_PLIST)
	$(call wait_for_notarization)
	@echo "Notarization completed, downloading results..."
	$(VERB) $(CURL) -o $(AUDIT_INFO_JSON) `$(PLIST_BUDDY) \
	    -c "Print :notarization-info:LogFileURL" $(REQUEST_INFO_PLIST)`
	$(VERB) if [ `$(PLIST_BUDDY) -c "Print :notarization-info:Status" \
	    $(REQUEST_INFO_PLIST)` != "success" ]; then \
		echo "Notarization failed" >&2; \
		false; \
	fi
	@echo "Notarization succeeded"
	$(VERB) rm $(ZIPNAME)
