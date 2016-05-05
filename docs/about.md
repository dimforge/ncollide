# About

<table markdown="1">
<tr>
<td id="nostyle_table" style="vertical-align:middle">
<a href="https://github.com/sebcrozet"><img id="left_float_img" src="http://www.gravatar.com/avatar/24f8431a2a28f633cba972f1176921e4?s=125"></img></a>
</td>
<td id="nostyle_table">
As of today, this website and the **ncollide** project are mostly developed by
Sébastien Crozet (aka.  [sebcrozet](https://github.com/sebcrozet)), a French
Ph.D Student. His researches are about finding new methods to perform fast
collision detection between complex CAD models that include smooth trimmed
surfaces like NURBS.  Besides his fulltime job, he works on **ncollide** trying
to help making Rust a great language for the development of real-time
geometry-intensive applications. For more informations about his professional
background, check out his
[LinkedIn](https://www.linkedin.com/pub/s%C3%A9bastien-crozet/25/875/416/en)
profile.
</td>
</tr>
</table>

# How to contribute

Contribution are greatly appreciated and can be technical as well as financial.

### Technical contribution

You are of course welcome to contribute to the source code of **ncollide** and
to this website. Simply make sure to [open an
issue](https://github.com/sebcrozet/ncollide/issues) on GitHub if you intend to
perform a large contribution. This should prevent other people from stepping
silently on your toes and ensure your work is going to be merged when it is
ready.


#### Working on this website

You can contribute to this website by completing, improving, and correcting
it. Do not hesitate to correct even the smallest, insignificant detail
(especially English mistakes, including typography). We love nitpicking!  This
website is composed of a set of markdown files located on the
[gh-pages](https://github.com/sebcrozet/ncollide/tree/gh-pages) branch of the
main **ncollide** repository. It is compiled using
[MkDocs v0.15.3](http://www.mkdocs.org/). As explained in the next section,
you need to fork, fix, and create a pull request targeting the **gh-pages**
branch to make your contribution integrable into our code base. There are no
specific rules, except that all compilable code to generate illustrations must
be located on the `src` folder.  Downloadable examples have to be located on
the `examples` directory of the main project, i.e., on the `master` branch.


#### Working on the library

If you intend to work on the source code of **ncollide**, you should start by
[forking](https://help.github.com/articles/fork-a-repo) the
[repository](https://github.com/sebcrozet/ncollide). Once you are done making
modifications to your own copy of **ncollide**, you have to [create a pull
request](https://help.github.com/articles/creating-a-pull-request) targeting
the **master** branch so that your contribution can be reviewed, commented, and
eventually merged.


### Financial contribution

Donations made to the lead developer are also appreciated. However, do not
forget that **donating is not a requirement**. You are and will always be free
to use **ncollide** for any purpose, including commercial applications, without
paying anything (see the [BSD-3
licence](https://github.com/sebcrozet/ncollide/blob/master/LICENSE)).
<div style="float:right">
<table style="border-style:none" align="center">
<tr style="border-style:none">
<td style="border-style:none">
<a href="https://flattr.com/submit/auto?user_id=sebcrozet&url=http%3A%2F%2Fncollide.org&title=ncollide&description=Rust+collision+detection+library." target="_blank"><img src="https://api.flattr.com/button/flattr-badge-large.png" alt="Flattr this" title="Flattr this" border="0"></a>
</td>
<td style="border-style:none">
<center>
<form action="https://www.paypal.com/cgi-bin/webscr" method="post" target="_blank">
<input type="hidden" name="cmd" value="_s-xclick">
<input type="hidden" name="encrypted" value="-----BEGIN PKCS7-----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-----END PKCS7-----
">
<input type="image" src="https://www.paypalobjects.com/en_US/i/btn/btn_donate_SM.gif" border="0" name="submit" alt="PayPal - The safer, easier way to pay online!">
</form>
</tr>
</center>
</td>
</table>
</div>

# Contributors

Here are the known contributors to the **ncollide** project, sorted in
alphabetical order:

* [Alexander Mistakidis](https://github.com/aamistak)
* [Ben Foppa](https://github.com/bfops)
* [Brandon Waskiewicz](https://github.com/brandonw)
* [Clément Renault](https://github.com/Kerosene2000)
* [Corey Richardson](https://github.com/cmr)
* [Dabo Ross](https://github.com/daboross)
* [Eduard Bopp](https://github.com/aepsil0n)
* [Felix Rath](https://github.com/Futile)
* [Jake Kerr](https://github.com/jakerr)
* Jan Nockemann
* [Jeroen Bollen](https://github.com/Binero)
* [John P Mayer](https://github.com/johnpmayer)
* [Matt Brennan](https://github.com/quarterto)
* [Michael Riss](https://github.com/MichaelRiss)
* [Peter Nguyen](https://github.com/artichokes)
* [Smith Wendell](https://github.com/wackywendell)
* [Sven Nilsen](https://github.com/bvssvni)
* [Wesley Powell](https://github.com/wspowell)


If your name should or should not be on this list, or if it should be linked
with another website, please send an [email](mailto:developer@crozet.re) or
open an issue on [GitHub](https://github.com/sebcrozet/ncollide/issues).

